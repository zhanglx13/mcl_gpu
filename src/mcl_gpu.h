#ifndef MCL_GPU_H_
#define MCL_GPU_H_

#include <cuda.h>
#include <curand_kernel.h>
#include "range_libc/RangeLib.h"

class MCLGPU
{
public:
    MCLGPU(int num_particles);
    MCLGPU(){printf("Default constructor of MCLGPU called\n");}
    ~MCLGPU();

    void init_constants(float motion_dispersion_x,
                        float motion_dispersion_y,
                        float motion_dispersion_theta,
                        double inv_squash_factor);
    void set_angles(float *angles);
    void update(float *px, float *py, float *pangle,
                float *odom_delta, float *obs, int num_angles,
                double *weights);
    void set_sensor_table(double *sensorTable, int t_w);
    void set_map(ranges::OMap omap, float max_range);
protected:
    int np_; // number of particles
    float *d_particles_;
    double *d_weights_;
    float *d_odom_delta_;
    float *d_obs_;
    /* sensor table used by the sensor model */
    double *d_sensorTable_;
    int table_width_;
    /* distance map info */
    float *d_distMap_;
    int width_;
    int height_;
    float max_range_;

    curandState *d_states_;
};


#endif
