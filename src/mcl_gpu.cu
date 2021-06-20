#include "mcl_gpu.h"

extern void err_check();

#define CUDA_CALL(x) do { if((x) != cudaSuccess) {              \
            printf("Error at %s:%d\n",__FILE__,__LINE__);       \
            break;}} while(0)
            //return EXIT_FAILURE;}} while(0)

/* Constants stored in CUDA constant memory */
/* motion model dispersion constants */
__constant__ float c_motion_dispersion_x;
__constant__ float c_motion_dispersion_y;
__constant__ float c_motion_dispersion_theta;

/* cuRAND state setup kernel */
__global__ void setup_kernel(curandState *state)
{
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    /* Each thread gets same seed, a different sequence
       number, no offset */
    curand_init(1234, id, 0, &state[id]);
}

__global__ void cuda_motion_sensor_model(
    curandState *states, float *particles, float *odom_delta,
    double *weights,
    int N)
{
    int p = threadIdx.x + blockIdx.x * blockDim.x;
    curandState localState = states[p];
    float angle = particles[p + N*2];
    /* apply motion model to each particle */
    float local_delta_x = cosf(angle) * odom_delta[0] - sinf(angle) * odom_delta[1];
    float local_delta_y = sinf(angle) * odom_delta[0] + cosf(angle) * odom_delta[1];
    float2 rnn2 = curand_normal2(&localState);
    float  rnn  = curand_normal(&localState);
    particles[p]   += local_delta_x + rnn2.x * c_motion_dispersion_x;
    particles[p+N] += local_delta_y + rnn2.y * c_motion_dispersion_y;
    particles[p+N*2] += angle +       rnn *    c_motion_dispersion_theta;

    states[p] = localState;
}

MCLGPU::MCLGPU(int num_particles): np_(num_particles)
{
    printf("MCLGPU constructor called!! Number of particles: %d\n", np_);

    /* Allocated device space */
    CUDA_CALL(cudaMalloc((void**)&d_particles_, sizeof(float)*np_*3));
    CUDA_CALL(cudaMalloc((void**)&d_weights_, sizeof(double)*np_));

    /* Initialize curand */
    CUDA_CALL(cudaMalloc((void **)&d_states, np_*sizeof(curandState)));
    setup_kernel<<<np_/NUM_THREADS, NUM_THREADS>>>(d_states);
}

MCLGPU::~MCLGPU()
{
    CUDA_CALL(cudaFree(d_particles_));
    CUDA_CALL(cudaFree(d_weights_));
}

void MCLGPU::init_constants(
    float motion_dispersion_x, float motion_dispersion_y, float motion_dispersion_theta)
{
    CUDA_CALL(cudaMemcpyToSymbol(c_motion_dispersion_x,
                                 &motion_dispersion_x, sizeof(c_motion_dispersion_x)));
    CUDA_CALL(cudaMemcpyToSymbol(c_motion_dispersion_y,
                                 &motion_dispersion_y, sizeof(c_motion_dispersion_y)));
    CUDA_CALL(cudaMemcpyToSymbol(c_motion_dispersion_theta,
                                 &motion_dispersion_theta, sizeof(c_motion_dispersion_theta)));
}

void MCLGPU::update(float *px, float *py, float *pangle,
                    float *odometry_delta,
                    double *weights)
{
    printf("MCLGPU::update() is called\n");
    /* Copy data from host to device */
    CUDA_CALL(cudaMemcpy(d_particles_, px, sizeof(float)*np_, cudaMemcpyHostToDevice));
    CUDA_CALL(cudaMemcpy(d_particles_+np_, py, sizeof(float)*np_, cudaMemcpyHostToDevice));
    CUDA_CALL(cudaMemcpy(d_particles_+np_*2, pangle, sizeof(float)*np_, cudaMemcpyHostToDevice));

    float *d_odom_delta;
    CUDA_CALL(cudaMalloc((void**)&d_odom_delta, sizeof(float)*3));
    CUDA_CALL(cudaMemcpy(d_odom_delta, odometry_delta, sizeof(float)*3, cudaMemcpyHostToDevice));

    cuda_motion_sensor_model<<<np_/NUM_THREADS, NUM_THREADS>>>(
        d_states, d_particles_, d_odom_delta,
        d_weights_,
        np_);

    CUDA_CALL(cudaMemcpy(weights, d_weights_, sizeof(double)*np_, cudaMemcpyDeviceToHost));
}