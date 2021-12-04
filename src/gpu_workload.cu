#include "gpu_workload.h"
#include <cstdlib>
#include <ctime>

GPU_Workload::GPU_Workload(int sz) : sz_(sz)
{
    srand(time(NULL));
    input = (int*)malloc(sizeof(int) * sz_ );
    for (int i = 0; i < sz_ ; i++)
        input[i] = rand() % 100;
    output = (int*)malloc(sizeof(int) * sz_ );
    cudaMalloc((void**)&d_input, sizeof(int)*sz_);
    cudaMalloc((void**)&d_output, sizeof(int)*sz_);
}

GPU_Workload::~GPU_Workload()
{
    free(input);
    free(output);
    cudaFree(d_input);
    cudaFree(d_output);
}

__device__ int d_op(int x)
{
    float tmp;
    for (int i = 0 ; i < 5000; i ++){
        tmp = 1.0 / cosf((float)x);
        tmp = 1.0 / sinf(tmp);
    }
    return (int)tmp;
}

__global__ void myKernel(int* d_input, int* d_output, int N)
{
    unsigned int tid = threadIdx.x + blockDim.x * blockIdx.x;
    if (tid >= N ) return ;
    d_output[tid] = d_op(d_input[tid]);
}

void GPU_Workload::callKernel()
{
    int threads = 256;
    // int blocks = (sz_ + threads - 1) / threads;
    int blocks = 7;
    cudaMemcpy(d_input, input, sizeof(int)*sz_, cudaMemcpyHostToDevice);
    myKernel<<<blocks, threads>>>(d_input, d_output, sz_);
    cudaMemcpy(output, d_output, sizeof(int)*sz_, cudaMemcpyDeviceToHost);
    cudaStreamSynchronize(cudaStreamPerThread);
}