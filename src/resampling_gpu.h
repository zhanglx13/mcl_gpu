#ifndef _RESAMPLING_GPU_H_
#define _RESAMPLING_GPU_H_

#include "errorCheckingMacro.cuh"

class ResamplingGPU
{
public:
    ResamplingGPU(int NP, int dim);
    ~ ResamplingGPU();

    void setParticles(float *px, float *py, float *pz);
    void getParticles(float *px, float *py, float *pz);
    void setWeights(double *w);
    void getWeights(double *w);
    void doSystematicRes(float *px, float *py, float *pz, double *w);
private:
    float * d_particles_;
    float * d_particles_new_;
    int NP_;
    int dim_;
    double * d_w_;
    double * d_w_normalized_;
    double ** g_scan_;
    int *d_indexA_;
    int level_ {0};
};


#endif
