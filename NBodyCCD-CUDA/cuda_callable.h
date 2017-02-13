#pragma once
//Use this CUDA_CALLABLE_MEMBER affix for functions called by device codes.
//Usage:
//CUDA_CALLABLE_MEMBER void foo()
//{
//	//todo
//}
#ifdef __CUDACC__
#define CUDA_CALLABLE_MEMBER __host__ __device__
#else
#define CUDA_CALLABLE_MEMBER
#endif 