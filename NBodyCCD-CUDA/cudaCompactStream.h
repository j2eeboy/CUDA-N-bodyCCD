#pragma once

#include <thrust/scan.h>
#include <thrust/copy.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include "Cuda_Support.h"
#include "helper_math.h"






// this is a special kind of array on the GPU memory
// that contains a lot of subarrays
// every subarray has its own size and capacity
// can be compacted using prefix sum
template <class T>
class CompactStream
{
public:

	// the actual space that stores every elements
	T* _mainarr_d;

	// boundaries of subarrays.
	// subarr[i] ranges from [ _boundaries[i-1], _boundaries[i] )
	// with this we can get the indices into each subarray
	uint* _boundaries_d;	

	// occupied size of every subarray
	// subarr[i] has first _sizes[i] elements occupied
	uint* _sizes_d;

	// number of subarrays
// 	uint* _subarr_num_d;
	uint _subarr_num;

	// the flag indicating whether the stream is compacted or not.
	// usually a stream can only be compacted once.
	bool _compacted;

	// initialize the stream 
	// with subarr_num subarrays,
	// each with subarr_capacity as reserved space
	CompactStream(uint subarr_num, uint subarr_capacity);

// 	__host__ __device__ CompactStream(const CompactStream& rhs);

	~CompactStream();

// 	// get the subarray instance at [arr_idx]
// 	__device__ SubArr get_subarr_at(uint arr_idx);


	// get element at [arr_idx][val_idx]
	__device__ T& get_elem_at(uint arr_idx, uint val_idx);


	// push back element at subarray[arr_idx]
	// return 0 if unsuccessful
	__device__ int push_back_at(const T& val, uint arr_idx);

	// compact this stream using prefix sum
	__host__ int shrink_to_fit();

	// the actual initizlizer, for initializing from malloced pointer
	__host__ void init(uint subarr_num, uint subarr_capacity);

	// get the current size of main array
	__host__ __device__ uint size_mainarr();
};

template <class T>
__host__ __device__ uint CompactStream<T>::size_mainarr()
{
	return _boundaries_d[_subarr_num - 1];
}

template <class T>
__host__ void CompactStream<T>::init(uint subarr_num, uint subarr_capacity)
{
	//TODO: ifdef __CUDACC__ cumalloc

// 	_mainarr_d = cudaNewArray<T>(subarr_num*subarr_capacity);
// 	_boundaries_d = cudaNewArray<uint>(subarr_num);
	// 	_sizes_d = cudaNewArray<uint>();	
	cudaDeviceSynchronize();
	_compacted = false;
	gpuErrchk(cudaMallocManaged((void**)&_mainarr_d, sizeof(T)*subarr_num*subarr_capacity));
	gpuErrchk(cudaMallocManaged((void**)&_boundaries_d, sizeof(uint)*subarr_num));
	gpuErrchk(cudaMallocManaged((void**)&_sizes_d, sizeof(uint)*subarr_num));
// 	gpuErrchk(cudaMallocManaged((void**)&_subarr_num_d, sizeof(uint)));

// 	*_subarr_num_d = subarr_num;
	_subarr_num = subarr_num;
//	std::cout << "subarray num="<<_subarr_num << std::endl;
	gpuErrchk(cudaDeviceSynchronize());
	cudaMemset(_sizes_d, 0, sizeof(uint)*subarr_num);
	uint* _boundaries_h = new uint[subarr_num];

	for (int i = 0; i < subarr_num; i++)
	{
		_boundaries_h[i] = subarr_capacity*(i + 1);
	}
// 	_mainarr_len = _boundaries_h[subarr_num - 1];
	cudaMemcpy(_boundaries_d, _boundaries_h, sizeof(uint)*subarr_num, cudaMemcpyHostToDevice);
	delete[] _boundaries_h;

}

// template <class T>
// __host__ __device__ CompactStream<T>::CompactStream(const CompactStream& rhs)
// {
// 
// }

template <class T>
__global__
void kernelCompactStreamCopyMem(
T* _mainarr_d,
uint* _boundaries_d,
uint* _sizes_d,
uint _subarr_num,
T* new_mainarr_d,
uint* new_boundaries_d
)
{
	//copy contents
	uint index = blockDim.x * blockIdx.x + threadIdx.x; //index of this thread
	if (index >= _subarr_num) //return if invalid index 
		return;
	T *subarr_src, *subarr_dst;
	uint subarr_size;
	uint offset_dst = index == 0 ? 0 : new_boundaries_d[index - 1];
	uint offset_src = index == 0 ? 0 : _boundaries_d[index - 1];
	subarr_src = _mainarr_d + offset_src;
	subarr_dst = new_mainarr_d + offset_dst;
	subarr_size = _sizes_d[index];
	if (subarr_size!=0)
		memcpy(subarr_dst, subarr_src, subarr_size*sizeof(T));
}


template <class T>
CompactStream<T>::~CompactStream()
{
	gpuErrchk(cudaFree(_mainarr_d));
	gpuErrchk(cudaFree(_boundaries_d));
	gpuErrchk(cudaFree(_sizes_d));
// 	gpuErrchk(cudaFree(_subarr_num_d));
}

template <class T>
CompactStream<T>::CompactStream(uint subarr_num, uint subarr_capacity)
{
	init(subarr_num, subarr_capacity);
}


template <class T>
__device__ T& CompactStream<T>::get_elem_at(uint arr_idx, uint val_idx)
{
	uint arr_offset;
	arr_offset = (arr_idx == 0) ? 0 : _boundaries_d[arr_idx - 1];
	return _mainarr_d[arr_offset + val_idx];
}

template <class T>
__device__ int CompactStream<T>::push_back_at(const T& val, uint arr_idx)
{
	uint arr_offset;
	uint& arr_size = _sizes_d[arr_idx];
	arr_offset = (arr_idx == 0) ? 0 : _boundaries_d[arr_idx - 1];
	assert(arr_offset + arr_size < _boundaries_d[arr_idx]);
	_mainarr_d[arr_offset + arr_size] = val;
	arr_size++;
	return 1;
}

template <class T>
int CompactStream<T>::shrink_to_fit()
{
	gpuErrchk(cudaDeviceSynchronize());



	// Restrit to compact only once.
	if (_compacted)
	{
		printf("Stream already compressed.\n");
		return -1;
	} 
	else
	{
		_compacted = true;
	}

	//status of previous cuda command
	cudaError_t cuda_status;
	// prefix sums of sizes, use as new boundary.
	uint* new_boundaries_d;
	gpuErrchk(cudaMallocManaged((void**)&new_boundaries_d, sizeof(uint)*(_subarr_num)));

	gpuErrchk(cudaDeviceSynchronize());
	// wrap raw pointer with a device_ptr for prefix sum
	thrust::device_ptr<uint> _sizes_d_wrapped_begin(_sizes_d);
	thrust::device_ptr<uint> _sizes_d_wrapped_end(&(_sizes_d[_subarr_num]));
	thrust::device_ptr<uint> new_boundaries_d_wrapped(new_boundaries_d);

	gpuErrchk(cudaDeviceSynchronize());
	// use prefix sum to get the new boundaries from previous array sizes
	thrust::inclusive_scan(_sizes_d_wrapped_begin, _sizes_d_wrapped_end, new_boundaries_d_wrapped);
	gpuErrchk(cudaDeviceSynchronize());
	// create a new mainarray for the stream
	T* new_mainarr_d;
	uint _mainarr_len = new_boundaries_d[_subarr_num - 1];
	gpuErrchk(cudaMallocManaged((void**)&new_mainarr_d, sizeof(T)*_mainarr_len));
	// invoke the kernel to copy the content from the old main array to the new one
	kernelCompactStreamCopyMem << <1 + (_subarr_num) / 256, 256 >> >(
		_mainarr_d,_boundaries_d,_sizes_d,_subarr_num,
		new_mainarr_d, new_boundaries_d
		);
	cudaDeviceSynchronize();
	gpuErrchk(cudaGetLastError());

	// destroy the old boundaries and mainarray
	gpuErrchk(cudaFree(_boundaries_d));
	gpuErrchk(cudaFree(_mainarr_d));
	// and use the new ones just generated.
	_boundaries_d = new_boundaries_d;
	_mainarr_d = new_mainarr_d;
	cudaDeviceSynchronize();

//	std::cout << "stream after compact:\n"
//		<< "mainarr len=" << this->size_mainarr() << '\n'
//		<< "subarr num=" << this->_subarr_num << std::endl;

	return 1;
}

