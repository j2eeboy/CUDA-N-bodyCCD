#include "broad_phase_culling.h"


//we expect less than SNP_RATE ratio objects to collide on one axis.
// that is, the result array at the length of SNP_RATE*num_of_bodies need to be assigned for each sweep and prune thread. 
float SNP_RATE = 0.15f;
// the probability of a pair of object after sweep and prune would further pass the overlap test
float OVERLAP_RATE = 0.01f;

extern void test(KernelArray<Result2B> collision_pair_arr,
	KernelArray<AABB> aabb_arr,
	KernelArray<uint> aabb_map_arr, mesh &mesh1, bvh &bvh1);




// the kernel that
// generates a map from aabb.id to index of aabb
// i.e., aabb_arr[map_id_idx[id]].id=id
__global__
void kernelMakeAABBMapId2Idx(
KernelArray<AABB> aabb_arr,
KernelArray<uint> aabb_map_arr
)
{
	uint idx = blockDim.x*blockIdx.x + threadIdx.x;
	uint size = aabb_arr._size;
	if (idx >= size) return;	//return if thread id beyond size

	AABB& aabb_d = aabb_arr[idx];
	uint id = aabb_d.id;
	aabb_map_arr[id] = idx;
}



// the kernel that
// does sweep and prune on one sorted axis
// and put every possible collision into result stream.
__global__
void kernelParallelSnP(
KernelArray<uint> endpoint_idx_arr,
KernelArray<EndPoint> endpoint_arr,
CompactStream<Result2B>* result,
KernelArray<AABB> aabb_arr,
KernelArray<uint> aabb_map_arr
)
{
	uint idx = blockDim.x*blockIdx.x + threadIdx.x;
	uint endpoint_arr_size = endpoint_arr._size;
	if (idx >= endpoint_arr_size) return;	//return if thread id beyond index

	uint endpoint_idx = endpoint_idx_arr[idx];
	const EndPoint& endpoint = endpoint_arr[endpoint_idx];
	uint aabb_id = endpoint.id;
	// 	printf("Thread %d: <%d,??>\n", idx, aabb_id);
	if (endpoint.minmax == ENDPOINT_MAX) return;	//only start sweeping from MIN endpoint
	uint result_subarr_idx = endpoint_idx / 2;		//lock-free subarr. endpoint_idx/2 for endpoint_num=2*aabb_num
	uint idx_it = idx + 1;
	while (true)
	{
		endpoint_idx = endpoint_idx_arr[idx_it];
		EndPoint& endpoint_it = endpoint_arr[endpoint_idx];
		uint aabb_id_it = endpoint_it.id;
		assert(endpoint_idx < endpoint_arr_size);
		if (aabb_id == aabb_id_it)
		{
			break;
		}
		// put Cell* =nullptr HERE

		AABB& lhs = aabb_arr[aabb_map_arr[aabb_id]];
		AABB& rhs = aabb_arr[aabb_map_arr[aabb_id_it]];


		if (endpoint_it.minmax == ENDPOINT_MIN)
			if (lhs.overlaps(rhs))
			{
				Result2B res(NULL, aabb_id, aabb_id_it);
				result->push_back_at(res, result_subarr_idx);
			}
		// 		printf("Thread %d: <%d,%d>\n", idx, aabb_id, aabb_id_it);
		idx_it++;

	}
}



__global__
void kernelExractKeyForResult2B(
	KernelArray<Result2B> result2b_arr,
	KernelArray<uint64_t> result2b_key_arr
)
{
	uint idx = blockDim.x*blockIdx.x + threadIdx.x;
	uint arr_size = result2b_arr._size;
	if (idx >= arr_size) return;	//return if thread id beyond index
	Result2B& res = result2b_arr[idx];
	uint key_first, key_second;
	key_first = res.fid(0);
	key_second = res.fid(1);
	uint64_t key;
	key = key_first;
	key<<= 32;
	key += key_second;
	result2b_key_arr[idx] = key;
}

// constructor from list<AABB>
cudaBroadPhaseCullingSession::cudaBroadPhaseCullingSession(const list<AABB*> & aabb_list) :
aabb_vector_d(aabb_list.size()),
endpoint_vector_d(aabb_list.size() * 2),
endpoint_x_val_vector_d(aabb_list.size() * 2),
endpoint_x_idx_vector_d(aabb_list.size() * 2)
{
}

/*

updateTimeStep

initAABB
list<AABB> -> device_vector<AABB>

[SpacialSubdivide]

SweepAndPrune
*/

std::list<Result2B> cudaBroadPhaseCullingSession::BroadPhaseCulling(mesh &mesh1, bvh &bvh1)
{
	std::list<Result2B> ret_val;

	// sort indices of endpoints
	thrust::sort_by_key(endpoint_x_val_vector_d.begin(), endpoint_x_val_vector_d.end(), endpoint_x_idx_vector_d.begin());

	// create a CompactStream to contain Result2B
	CompactStream<Result2B> * result_stream_d;
	gpuErrchk(cudaMallocManaged((void**)&result_stream_d, sizeof(CompactStream<Result2B>)));
	
	// init(number of subarrays in the stream, size of each subarray). 
	// +5 because when number of aabb is too small, aabb_vector_d.size()*SNP_RATE may become too small or even 0
	result_stream_d->init(aabb_vector_d.size(), aabb_vector_d.size()*SNP_RATE+5);	

	// make a map aabb.id-> index of aabb
	// for the convenience of parallel snp
	uint result_size = result_stream_d->size_mainarr();
	thrust::device_vector<uint> map_id_idx_d(result_size, UINT_MAX);
	kernelMakeAABBMapId2Idx << <result_size / MAX_BLOCK_DIM_X + 1, MAX_BLOCK_DIM_X >> >(aabb_vector_d, map_id_idx_d);


	// execute parallel sweep and prune and put every Result2B into the stream
	// the stream is compacted using prefix sum at the end of the operation
	kernelParallelSnP << <this->endpoint_vector_d.size() / MAX_BLOCK_DIM_X + 1, MAX_BLOCK_DIM_X >> >(
		endpoint_x_idx_vector_d, endpoint_vector_d, result_stream_d, aabb_vector_d, map_id_idx_d
		);

	cudaDeviceSynchronize();

	//compact the stream to save memory
	// and make it easier for following operations.
	result_stream_d->shrink_to_fit();

	result_size = result_stream_d->size_mainarr();
	test(KernelArray<Result2B>(result_stream_d->_mainarr_d, result_size), aabb_vector_d, map_id_idx_d, mesh1, bvh1);


	cudaDeviceSynchronize();

	
	uint mainarr_size = result_stream_d->size_mainarr();
	
	// dump every Result2B from the stream to the host
	thrust::host_vector<Result2B> result_h(
		thrust::device_ptr<Result2B>(result_stream_d->_mainarr_d),
		thrust::device_ptr<Result2B>(result_stream_d->_mainarr_d + mainarr_size));

	cudaDeviceSynchronize();
	// destroy the stream
	result_stream_d->~CompactStream();
	cudaFree(result_stream_d);
	
	//convert from thrust::host_vector to std::list
	for (thrust::host_vector<Result2B>::iterator val = result_h.begin(); val != result_h.end();val++)
	{
		if ((*val).fid(0) != UINT_MAX)
		{
			ret_val.push_back(Result2B((*val).cell, (*val).fid(0), (*val).fid(1)));
		}
	}
	
	return ret_val;
}





void cudaBroadPhaseCullingSession::updateAABB(list<AABB*> & aabb_list)
{
	//init endpoint_ptr which points to endpoints.
	int endpoint_num = aabb_list.size() * 2;
	std::vector<uint> endpoint_x_idx_vector_h(endpoint_num);
	for (unsigned int endpoint_idx = 0; endpoint_idx < endpoint_num; endpoint_idx++)
	{
		// init x,y,z direction endpoint ptr array (only need once)
		endpoint_x_idx_vector_h[endpoint_idx] = endpoint_idx;
	}
	endpoint_x_idx_vector_d = endpoint_x_idx_vector_h;



	std::vector<AABB> aabb_vector_h(aabb_list.size());
	std::vector<EndPoint> endpoint_vector_h(endpoint_num);
	std::vector<float> endpoint_x_val_vector_h(endpoint_num);

	// first update everything into temporary host_vectors
	//probably use kernel to speed up
	int i = 0;
	for (list<AABB *>::iterator it = aabb_list.begin(); it != aabb_list.end(); it++)
	{
		AABB& aabb_curr = **it;
		int endpoint_idx = i * 2;	
		int aabb_idx = i;

		//update aabb_vector
		AABB& aabb_curr_h = aabb_vector_h[aabb_idx];
		aabb_curr_h = aabb_curr;
		aabb_curr_h.lo = thrust::raw_pointer_cast(&endpoint_vector_d[endpoint_idx]);
		aabb_curr_h.hi = thrust::raw_pointer_cast(&endpoint_vector_d[endpoint_idx + 1]);

		//update endpoint_vector
		endpoint_vector_h[endpoint_idx] = *(aabb_curr.lo);
		endpoint_vector_h[endpoint_idx + 1] = *(aabb_curr.hi);
		

		//update endpoint_x,y,z_vector
		endpoint_x_val_vector_h[endpoint_idx] = aabb_curr.lo->val[0];
		endpoint_x_val_vector_h[endpoint_idx + 1] = aabb_curr.hi->val[0];

		i++;
	}
	//then dump everything to device_vector
	cudaDeviceSynchronize();
	aabb_vector_d = aabb_vector_h;
	endpoint_vector_d = endpoint_vector_h;
	endpoint_x_val_vector_d = endpoint_x_val_vector_h;

	// avoid deleting device ptr in host_vector destructor
	for (vector<AABB>::iterator it = aabb_vector_h.begin(); it != aabb_vector_h.end();it++)
	{
		(*it).hi = NULL;
		(*it).lo = NULL;
	}
}