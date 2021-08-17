#include "scanFan_no_mm.h"

template <typename T>
void scanFan_no_mm(T *out, T *in, size_t N, int b,  T **g_scanBlockSums)
{
    scanFan<T>(out, in, N, b, 0, g_scanBlockSums);
}

template
void scanFan_no_mm<int>(int *out, int *in, size_t N, int b, int **g_scanBlockSums);
template
void scanFan_no_mm<float>(float *out, float *in, size_t N, int b, float **g_scanBlockSums);
template
void scanFan_no_mm<double>(double *out, double *in, size_t N, int b, double **g_scanBlockSums);