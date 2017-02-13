#pragma once

#include "Cuda_Support.h"
#include "result2b.h"
#include "myfeature.h"
#include "cudaCompactStream.h"
#include "mesh.h"
#include "bvh.h"

//#include <thrust/system/cuda/detail/detail/stable_radix_sort.h>
// 
/***************************
//Prefix Sum Examples:
#include <thrust/scan.h>
int data[6] = {1, 0, 2, 2, 1, 3};
thrust::inclusive_scan(data, data + 6, data); // in-place scan
// data is now {1, 1, 3, 5, 6, 9}
OR
int data[6] = {1, 0, 2, 2, 1, 3};
thrust::exclusive_scan(data, data + 6, data); // in-place scan
// data is now {0, 1, 1, 3, 5, 6}

- See more at: http://docs.nvidia.com/cuda/thrust/#sthash.XpPTbKra.dpuf
***************************/


// some values for kernel launching
const int MAX_BLOCK_DIM_X = 512;

const int THREAD_PER_BLOCK = 256;


class cudaBroadPhaseCullingSession
{

	// vector of all AABBs in the device memory
	thrust::device_vector<AABB> aabb_vector_d;

	//////////////////////////////////////////////////////////////////////////
	// vector of all current endpoints and corresponding arrays
	// are arranged in this way:
	// {
	//	aabb_vector_d[0].lo,
	// 	aabb_vector_d[0].hi,
	// 	aabb_vector_d[1].lo
	//  ....
	// }
	//////////////////////////////////////////////////////////////////////////
	thrust::device_vector<EndPoint> endpoint_vector_d;

	//////////////////////////////////////////////////////////////////////////
	// endpoint_val --> endpoint_idx --> endpoint
	// (sorting key)    (sorting val)    (actual target)
	// this is for utilizing thrust::sort_by_key()
	// which is a radix sort.
	//////////////////////////////////////////////////////////////////////////
	// x,y,z component values of endpoints, serves as sorting key
	//////////////////////////////////////////////////////////////////////////
	thrust::device_vector<float> endpoint_x_val_vector_d;
// 	thrust::device_vector<float> endpoint_y_val_vector_d;
// 	thrust::device_vector<float> endpoint_z_val_vector_d;


	//endpoints sorted in x,y,z directions, serves as sorting value
	thrust::device_vector<uint> endpoint_x_idx_vector_d;
// 	thrust::device_vector<uint> endpoint_y_idx_vector_d;
// 	thrust::device_vector<uint> endpoint_z_idx_vector_d;

	


public:
	// initialize a culling session
	// for reuse
	cudaBroadPhaseCullingSession(const list<AABB*> & aabb_list);

	// iterate to the next frame
	// do parallel spacial subdivision,
	// sweep and prune
	std::list<Result2B> BroadPhaseCulling(mesh &, bvh &);

	//update AABB list from host.
	//<del>currently we assume number of AABB doesn't change</del>
	void updateAABB(list<AABB*> & aabb_list);

	//calculate the spacial hash
	//void calcHash();

	//iterate to the next step
	//void nextTimeStep();

// 	void spacialSubdivision(const cudaExtent& spacialSubdivisionSize, CompactStream<Result2B>* result);

	// parallel sweep and prune operation
	// execute parallel sweep and prune and put every Result2B into the stream
	// the stream is compacted using prefix sum at the end of the operation
// 	void parallelSnP(CompactStream<Result2B>* result);

	// test every Result2B from the previous result
	// and invalidate the ones that are not colliding.
// 	void parallelOverlapTest(CompactStream<Result2B>* result, mesh &mesh1, bvh &bvh1);



};