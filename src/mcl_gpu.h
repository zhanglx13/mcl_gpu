#ifndef MCL_GPU_H_
#define MCL_GPU_H_

#include <cuda.h>
#include <curand_kernel.h>
#include "range_libc/RangeLib.h"

class MCLGPU
{
public:
    MCLGPU(int num_particles);
    MCLGPU(){printf("Default constructor of MCLGPU called\n");};
    ~MCLGPU();

    void init_constants(float motion_dispersion_x, float motion_dispersion_y, float motion_dispersion_theta);

    void update(float *px, float *py, float *pangle,
                float *odom_delta,
                double *weights);
protected:
    int np_; // number of particles
    float *d_particles_;
    double *d_weights_;
    curandState *d_states;
};


#endif
