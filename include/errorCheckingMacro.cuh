/*
 * https://codingbyexample.com/2019/02/06/error-handling-for-gpu-cuda/
 */
// errorCheckingMacro.cuh
#ifndef CHECK_CUDA_ERROR_M_H
#define CHECK_CUDA_ERROR_M_H

#define PRINT_ON_SUCCESS 0

#define H2D cudaMemcpyHostToDevice
#define D2H cudaMemcpyDeviceToHost

// To be used around calls that return an error code, ex. cudaDeviceSynchronize or cudaMallocManaged
static void checkError(cudaError_t code, char const * func, const char *file, const int line, bool abort = true);
#define checkCUDAError(val) { checkError((val), #val, __FILE__, __LINE__); }    // in-line regular function

// To be used after calls that do not return an error code, ex. kernels to check kernel launch errors
static void checkLastError(char const * func, const char *file, const int line, bool abort = true);
#define checkLastCUDAError(func) { checkLastError(func, __FILE__, __LINE__); }
#define checkLastCUDAError_noAbort(func) { checkLastError(func, __FILE__, __LINE__, 0); }

static void checkError(cudaError_t code, char const * func, const char *file, const int line, bool abort)
{
    if (code != cudaSuccess)
    {
        const char * errorMessage = cudaGetErrorString(code);
        fprintf(stderr, "CUDA error returned from \"%s\" at %s:%d, Error code: %d (%s)\n", func, file, line, code, errorMessage);
        if (abort){
            cudaDeviceReset();
            exit(code);
        }
    }
    else if (PRINT_ON_SUCCESS)
    {
        const char * errorMessage = cudaGetErrorString(code);
        fprintf(stderr, "CUDA error returned from \"%s\" at %s:%d, Error code: %d (%s)\n", func, file, line, code, errorMessage);
    }
}

static void checkLastError(char const * func, const char *file, const int line, bool abort)
{
    cudaError_t code = cudaGetLastError();
    if (code != cudaSuccess)
    {
        const char * errorMessage = cudaGetErrorString(code);
        fprintf(stderr, "CUDA error returned from \"%s\" at %s:%d, Error code: %d (%s)\n", func, file, line, code, errorMessage);
        if (abort) {
            cudaDeviceReset();
            exit(code);
        }
    }
    else if (PRINT_ON_SUCCESS)
    {
        const char * errorMessage = cudaGetErrorString(code);
        fprintf(stderr, "CUDA error returned from \"%s\" at %s:%d, Error code: %d (%s)\n", func, file, line, code, errorMessage);
    }
}

#endif // CHECK_CUDA_ERROR_M_H
