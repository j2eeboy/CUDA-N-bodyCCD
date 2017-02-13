//the header including some supports and utils for cuda
#pragma once
#ifndef __CUDACC__
//#define __CUDACC__
#endif





#include <cuda.h>
#include <device_functions.h>
#include <cuda_runtime_api.h>
#include <host_defines.h>
#include <cstdio>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/sort.h>
#include <thrust/scan.h>
#include <thrust/copy.h>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "helper_math.h"
#include "cuda_callable.h"

// Template structure to pass to kernel
template < typename T >
struct KernelArray
{
	T* _array;
	unsigned int _size;

	// constructor allows for implicit conversion
	KernelArray(thrust::device_vector<T>& dVec) {
		_array = thrust::raw_pointer_cast(&dVec[0]);
		_size = (unsigned int)dVec.size();
	}

	// constructor from raw device pointer
	KernelArray(T* arr_d, unsigned int size) 
		: _array(arr_d), _size(size)
	{}


	CUDA_CALLABLE_MEMBER T& operator[](const unsigned int idx)
	{
		return _array[idx];
	}
};
class Managed {
public:
	void *operator new(size_t len){
		void *ptr;
		cudaMallocManaged(&ptr, len);
		cudaDeviceSynchronize();
		return ptr;
	}

		void operator delete(void *ptr) {
		cudaDeviceSynchronize();
		cudaFree(ptr);
	}
};
// template<typename T>
// CUDA_CALLABLE_MEMBER
// T* cudaNewArray(const size_t size)
// {
// 	T* ret_val;
// #ifdef __CUDACC__
// 	cudaMalloc((void**)&ret_val,sizeof(T)*size);
// #else
// 	ret_val = new T[size];
// #endif
// 	return ret_val;
// }
// 
// template<typename T>
// CUDA_CALLABLE_MEMBER
// void cudaDeleteArray(T* arr)
// {
// #ifdef __CUDACC__
// 	cudaFree(arr);
// #else
// 	delete[] arr;
// #endif
// }

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort = true)
{
	if (code != cudaSuccess)
	{
		fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
		if (abort) exit(code);
	}
}

void inline testscan()
{
	cudaDeviceSynchronize();
	std::vector<unsigned int> vec_h(4000, 100);
	thrust::host_vector<unsigned int>  arr_h = vec_h;
	thrust::device_vector<unsigned int> arr_d = vec_h;

	thrust::inclusive_scan(arr_d.begin(), arr_d.end(), arr_d.begin());

	arr_h = arr_d;
	//	for (auto& val : arr_d)
	//	{
	//		std::cout << val << std::endl;
	//	}
	cudaDeviceSynchronize();

return;
}