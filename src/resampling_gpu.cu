
#include <stdio.h>
#include <time.h>
#include "resampling_gpu.h"

#include "scanFan_no_mm.h"

/*
 * Normalize a scanned array
 * The result array elements are in the range of [0, N] (not [0,1])
 */
template <typename T>
__global__ void normalize(T *array_new, T *array, size_t N)
{
    unsigned int tid = threadIdx.x + blockIdx.x * blockDim.x;
    __shared__ T sum;
    if(threadIdx.x == 0){
        sum = array[N-1];
        //printf(" thread %d is loading %e  N is %d\n", (int)tid, (double)sum, (int) N);
    }
    __syncthreads();
    if (tid < N)
        array_new[tid] = array[tid]/sum*(T)N;
}

/*
 * Search sp in array_normalized (also sorted) using binary search
 */
template <typename T>
__device__ int binSearch(T *array_normalized, size_t N, T sp)
{
    if (sp <= array_normalized[0])
        return 0;
    int low = 0;
    int high = N-1;
    int mid;
    while (low + 1 < high){
        mid = (low + high)/2;
        if (sp <= array_normalized[mid])
            high = mid;
        else
            low = mid;
    }
    return high;
}

/*
 * Pick N indices to represent the sampled elements.
 */
template <typename T>
__global__ void select(int *indexA, T *array_normalized, size_t N, T rand_mu)
{
    unsigned int tid = threadIdx.x + blockIdx.x * blockDim.x;
    T sp = (T)tid + rand_mu;
    if (tid < N)
        indexA[tid] = binSearch<T>(array_normalized, N, sp);
}

/*
 * Update particles with the picked indices
 */
template <typename T>
__global__ void update(T *particles_new, T *particles, int *indexA, size_t N, size_t sDim)
{
    unsigned int tid = threadIdx.x + blockIdx.x * blockDim.x;
    T tempVal;
    if (tid < N)
        for (int i = 0 ; i < sDim ; i ++){
            tempVal = particles[i*N + indexA[tid]];
            particles_new[tid + i*N] = tempVal;
    }
}

ResamplingGPU::ResamplingGPU(int NP, int dim):
    NP_(NP), dim_(dim)
{
    /* allocate space for partilces and its copy */
    unsigned int pp_sz = sizeof(float) * NP_ * dim_;
    checkCUDAError(cudaMalloc((void**)&d_particles_, pp_sz));
    checkCUDAError(cudaMalloc((void**)&d_particles_new_, pp_sz));

    /* allocate space for weight and normalized weight */
    unsigned int w_sz = sizeof(double) * NP_;
    checkCUDAError(cudaMalloc((void**)&d_w_, w_sz));
    checkCUDAError(cudaMalloc((void**)&d_w_normalized_, w_sz));

    /* allocate space for g_scan */
    level_ = 0;
    int numEle = NP_;
    if(NP_ <= NUM_THREADS)
        g_scan_ = nullptr;
    else{
        while(numEle > NUM_THREADS){
            level_ ++;
            numEle = numEle / NUM_THREADS;
        }
        g_scan_ = (double**) malloc(level_ * sizeof(double*));
        numEle = NP_;
        level_ = 0;
        while(numEle > NUM_THREADS){
            numEle = numEle / NUM_THREADS;
            checkCUDAError(cudaMalloc((void**) &g_scan_[level_],  numEle * sizeof(double)));
            level_ ++;
        }
    }

    /* allocate space for indexA */
    checkCUDAError(cudaMalloc((void**) &d_indexA_, sizeof(int)*NP_));
}

ResamplingGPU::~ ResamplingGPU()
{
    checkCUDAError(cudaFree(d_particles_));
    checkCUDAError(cudaFree(d_particles_new_));
    checkCUDAError(cudaFree(d_w_));
    checkCUDAError(cudaFree(d_w_normalized_));
    checkCUDAError(cudaFree(d_indexA_));
    if (NP_ > NUM_THREADS){
        for (int i = 0; i < level_; i++)
            checkCUDAError(cudaFree(g_scan_[i]));
        free((void**)g_scan_);
    }
}

void ResamplingGPU::setParticles(float *px, float *py, float *pz)
{
    unsigned int pp_sz = sizeof(float) * NP_;
    checkCUDAError(cudaMemcpy(d_particles_, px, pp_sz, H2D));
    checkCUDAError(cudaMemcpy(d_particles_ + NP_, py, pp_sz, H2D));
    checkCUDAError(cudaMemcpy(d_particles_ + 2*NP_, pz, pp_sz, H2D));
}

void ResamplingGPU::getParticles(float *px, float *py, float *pz)
{
    unsigned int pp_sz = sizeof(float) * NP_;
    checkCUDAError(cudaMemcpy(px, d_particles_new_, pp_sz, D2H));
    checkCUDAError(cudaMemcpy(py, d_particles_new_ + NP_, pp_sz, D2H));
    checkCUDAError(cudaMemcpy(pz, d_particles_new_ + 2*NP_, pp_sz, D2H));
    cudaStreamSynchronize(cudaStreamPerThread);
}

void ResamplingGPU::setWeights(double *w)
{
    unsigned int w_sz = sizeof(double) * NP_;
    checkCUDAError(cudaMemcpy(d_w_, w, w_sz, H2D));
}

void ResamplingGPU::getWeights(double *w)
{
    unsigned int w_sz = sizeof(double) * NP_;
    checkCUDAError(cudaMemcpy(w, d_w_, w_sz, D2H));
}

void ResamplingGPU::doSystematicRes(float *px, float *py, float *pz, double *w)
{
    /*
     * 1. Allocate device memory for particles and its copies, weight,
     *    and some helper arrays
     * 2. Copy particles and weights to device memory
     * 3. Call scanFan_no_mm to perform prefix scan on the weight array
     * 4. Call systematicRes
     */

    /* Step 1: memory allocation */
    /* Done at constructor */

    /* Step 2: copy data from host to device */
    setWeights(w);
    /* pack px, py, and pangle into continuous memory d_particles */
    setParticles(px, py, pz);

    /* Step 3: perform prefix scan on the weights */
    scanFan_no_mm(d_w_, d_w_, NP_, NUM_THREADS, g_scan_);

    /* Step 4: perform systematic resampling */
    srand(time(NULL));
    double rand_mu = (double)rand() / (double)RAND_MAX;
    unsigned int blocks = (NP_ + NUM_THREADS - 1 ) / NUM_THREADS;
    normalize<<<blocks, NUM_THREADS>>>(d_w_normalized_, d_w_, NP_);
    select<<<blocks, NUM_THREADS>>>(d_indexA_, d_w_normalized_, NP_, rand_mu);
    update<<<blocks, NUM_THREADS>>>(d_particles_new_, d_particles_, d_indexA_, NP_, dim_);

    /* Step 5: copy result back to host */
    getParticles(px, py, pz);
}