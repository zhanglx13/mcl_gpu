
#ifndef _SCAN_KERNEL_H_
#define _SCAN_KERNEL_H_

#include <stdio.h>
#include <math.h>
//#include "cudaUtils.h"

//*************************************************************************************************//
// Scan-Then-Fan
//*************************************************************************************************//

template<class T> 
inline __device__ T scanWarp(volatile T *sPartials)
{
	int tid = threadIdx.x;
	int lane = tid & 31;
	if (lane >= 1) sPartials[0] += sPartials[ - 1];
	if (lane >= 2) sPartials[0] += sPartials[ - 2];
	if (lane >= 4) sPartials[0] += sPartials[ - 4];
	if (lane >= 8) sPartials[0] += sPartials[ - 8];
	if (lane >= 16) sPartials[0] += sPartials[ - 16];

	return sPartials[0];
}

// template<class T>
// inline __device__ T scanWarp0(volatile T *sharedPartials)
// {
// 	int tid = threadIdx.x;
// 	sharedPartials[tid] += sharedPartials[tid - 1];
// 	sharedPartials[tid] += sharedPartials[tid - 2];
// 	sharedPartials[tid] += sharedPartials[tid - 4];
// 	sharedPartials[tid] += sharedPartials[tid - 8];
// 	sharedPartials[tid] += sharedPartials[tid - 16];

// 	return sharedPartials[tid];

// }

template<class T> 
inline __device__ T scanBlock(volatile T *sPartials)
{
    //extern __shared__ T warpPartials[];
    //T *warpPartials = SharedMemory<T>();
    extern __shared__ __align__(sizeof(T)) unsigned char my_smem[];
    T *warpPartials = reinterpret_cast<T *>(my_smem);

	int tid = threadIdx.x;
	int lane = tid & 31;
	int warpid = tid >> 5;

	T sum = scanWarp<T>(sPartials);
	__syncthreads();

	if(lane == 31)
		warpPartials[16 + warpid] = sum;
	__syncthreads();

	if(warpid == 0)
		scanWarp<T>(16 + warpPartials + tid);
		// scanWarp0<T>(warpPartials);
	__syncthreads();

	if(warpid > 0)
		sum += warpPartials[16 + warpid - 1];
	__syncthreads();

	*sPartials = sum;
	// sPartials[tid] = sum;
	__syncthreads();

	return sum;
}

template <class T , bool bWriteSpine> 
__global__ void scanAndWritePartials(T *out , T *gPartials, T *in, size_t N, size_t numBlocks)
{
    //T *sPartials = SharedMemoryV<T>();
    extern __shared__ __align__(sizeof(T)) unsigned char my_smem[];
    T *sPartials = reinterpret_cast<T *>(my_smem);

	int tid = threadIdx.x;
	volatile T *myShared = sPartials + tid;
	for(size_t iBlock = blockIdx.x ; iBlock < numBlocks ; iBlock += gridDim.x){
		size_t index = iBlock * blockDim.x + tid;

		*myShared = (index < N) ? in[index] : 0;
		__syncthreads();

		T sum = scanBlock(myShared);
		__syncthreads();

		if(index < N)
			out[index] = *myShared;

		if(bWriteSpine && (threadIdx.x == (blockDim.x - 1)))
			gPartials[iBlock] = sum;
	}
}

template<class T> 
__global__ void scanAddBaseSum(T *out , T *gBaseSums, size_t N, size_t numBlocks)
{
	int tid = threadIdx.x;
	T fan_value = 0;
	for(size_t iBlock = blockIdx.x ; iBlock < numBlocks ; iBlock += gridDim.x){
		size_t index = iBlock * blockDim.x + tid;
		if(iBlock > 0)
			fan_value = gBaseSums[iBlock - 1];
		out[index] += fan_value ;
	}
}


template <class T> 
void scanFan(T *out, T *in, size_t N, int b, int level,  T **g_scanBlockSums)
{
	if(N <= b){
		scanAndWritePartials<T, false><<<1,b,b*sizeof(T)>>>(out, 0, in, N, 1);
		return;
	}

	size_t numPartials = N/b;

	unsigned int maxBlocks = 2048;
	unsigned int numBlocks = (numPartials < maxBlocks) ? numPartials : maxBlocks;
	scanAndWritePartials<T, true><<<numBlocks,b,b*sizeof(T)>>>(out, g_scanBlockSums[level], in, N, numPartials);
	scanFan<T>(g_scanBlockSums[level], g_scanBlockSums[level], numPartials, b, level + 1 , g_scanBlockSums);
	scanAddBaseSum<T><<<numBlocks, b>>>(out, g_scanBlockSums[level], N, numPartials);
}

template
void scanFan<int>(int *out, int *in, size_t N, int b, int level,  int **g_scanBlockSums);
template
void scanFan<float>(float *out, float *in, size_t N, int b, int level,  float **g_scanBlockSums);
template
void scanFan<double>(double *out, double *in, size_t N, int b, int level,  double **g_scanBlockSums);


///
///  This wrapper function takes care of allocation and free of g_scanBlockSums
///  so that the API of the library becomes simpler. However, memory allocation
///  and deallocation time becomes part of the algorithm overhead.
///


// template <typename T>
// int scan_wrapper(T *out, T *in, size_t N, size_t block_size)
// {
//     T** g_scanBlockSums;
//     int level = 0;
//     int numEle = N;
//     while(numEle > block_size){
//         level ++;
//         numEle = numEle / block_size;
//     }
//     g_scanBlockSums = (T**) malloc(level * sizeof(T*));
//     numEle = N;
//     level = 0;
//     while(numEle > block_size){
//         numEle = numEle / block_size;
//         CUDA_CALL(cudaMalloc((void**) &g_scanBlockSums[level],  numEle * sizeof(T)));
//         level ++;
//     }
//     scanFan<T>(out, in, N, block_size, 0, g_scanBlockSums);

//     /* deallocate the memory */
//     for (int i = 0; i < level; i++)
//         CUDA_CALL(cudaFree(g_scanBlockSums[i]));
//     free((void**)g_scanBlockSums);
//     return 0;
// }

// template
// int scan_wrapper<double>(double *out, double *in, size_t N, size_t block_size);

// template
// int scan_wrapper<float>(float *out, float *in, size_t N, size_t block_size);

// template
// int scan_wrapper<int>(int *out, int *in, size_t N, size_t block_size);

// extern "C"
// void scanD(double *out, double *in, size_t N, size_t block_size)
// {
//     scan_wrapper<double>(out, in, N, block_size);
//     cudaDeviceSynchronize();
// }

// extern "C"
// void scanF(float *out, float *in, size_t N, size_t block_size)
// {
//     scan_wrapper<float>(out, in, N, block_size);
//     cudaDeviceSynchronize();
// }

// extern "C"
// void scanI(int *out, int *in, size_t N, size_t block_size)
// {
//     scan_wrapper<int>(out, in, N, block_size);
//     cudaDeviceSynchronize();
// }

#endif // #ifndef _SCAN_KERNEL_H_
