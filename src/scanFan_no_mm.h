/*************************************************************************************
    Author: Lixun Zhang
    Last Modified: Jan 25, 2017
*************************************************************************************

        This is the header file of scanFan_no_mm algorithm

        The API for doing scanFan is provided as a template function

        void scanFan_no_mm(T *out, T *in, size_t N, int b, T **g_scanBlockSums)

        @param out     output array with scanned values
        @param in      input array to be scanned
        @param N       number of elements in the array
        @param b       block size used when executing the kernel on GPU
        @param g_scan  a global 2D array to store the intermediate sum of each block

        The user has to provide all the input data and allocate space for the
        output data. For performance purposes, users better choose b to be a
        multiple of 32. This lib also provide two macros for allocating and
        de-allocating g_scan. The API is

        SCAN_ALLOC(T, g_scan, N, block_size)
        SCAN_FREE(g_scan)

        With these two macros, users do not need to use malloc and cudaMalloc to
        do memory management. However, there is only one constraint about using
        these two macros: The name given to the macro as the global 2D helper
        array (g_scan) must match in SCAN_ALLOC and SCAN_FREE.

        Note that the API DOES NOT call cudaDeviceSynchronize(). Therefore it is
        the user's responsibility to call such function to sync between host and
        device.

**************************************************************************************/

#ifndef SCANFAN_NO_MM_H
#define SCANFAN_NO_MM_H

template <typename T>
void scanFan(T *out, T *in, size_t N, int b, int level,  T **g_scan);

template <typename T>
void scanFan_no_mm(T *out, T *in, size_t N, int b, T **g_scan);

#define SCAN_ALLOC(T, g_scan, N, block_size)                            \
    int level = 0;                                                      \
    int numEle = N;                                                     \
    if(N <= block_size)                                                 \
        g_scan = nullptr;                                               \
    else{                                                               \
        while(numEle > block_size){                                     \
            level ++;                                                   \
            numEle = numEle / block_size;                               \
        }                                                               \
        g_scan = (T**) malloc(level * sizeof(T*));                      \
        numEle = N;                                                     \
        level = 0;                                                      \
        while(numEle > block_size){                                     \
            numEle = numEle / block_size;                               \
            checkCUDAError(cudaMalloc((void**) &g_scan[level],  numEle * sizeof(T))); \
            level ++;                                                   \
        }                                                               \
    }

#define SCAN_FREE(g_scan)                       \
    for (int i = 0; i < level_; i++)             \
        checkCUDAError(cudaFree(g_scan[i]));    \
    free((void**)g_scan);

#endif
