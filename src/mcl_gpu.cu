#include "mcl_gpu.h"
#include "errorCheckingMacro.cuh"

/* Constants stored in CUDA constant memory */
/* motion model dispersion constants */
__constant__ float c_motion_dispersion_x;
__constant__ float c_motion_dispersion_y;
__constant__ float c_motion_dispersion_theta;

__device__ void print_constants()
{
    printf("@@ Variables in constant memory:\n");
    printf("   c_motion_dispersion_x:     %f\n", c_motion_dispersion_x);
    printf("   c_motion_dispersion_y:     %f\n", c_motion_dispersion_y);
    printf("   c_motion_dispersion_theta: %f\n", c_motion_dispersion_theta);
}

/* cuRAND state setup kernel */
__global__ void setup_kernel(curandState *state)
{
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    if (id == 0) printf("@@ Kernel <<<setup_kernel>>> is called!\n");
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
    if (p == 0) {
        printf("@@ Kernel <<<cuda_sensor_model>>> is called!\n");
        print_constants();
    }
    curandState localState = states[p];
    float angle = particles[p + N*2];
    /* apply motion model to each particle */
    float local_delta_x = cosf(angle) * odom_delta[0] - sinf(angle) * odom_delta[1];
    float local_delta_y = sinf(angle) * odom_delta[0] + cosf(angle) * odom_delta[1];
    float2 rnn2 = curand_normal2(&localState);
    float  rnn  = curand_normal(&localState);
    particles[p]     += local_delta_x + rnn2.x * c_motion_dispersion_x;
    particles[p+N]   += local_delta_y + rnn2.y * c_motion_dispersion_y;
    particles[p+N*2] += angle +         rnn *    c_motion_dispersion_theta;

    states[p] = localState;
}

MCLGPU::MCLGPU(int num_particles): np_(num_particles)
{
    printf("## MCLGPU constructor called!! Number of particles: %d\n", np_);
    /* Allocated device space */
    checkCUDAError(cudaMalloc((void**)&d_particles_, sizeof(float)*np_*3));
    checkCUDAError(cudaMalloc((void**)&d_weights_, sizeof(double)*np_));

    /* Initialize curand */
    checkCUDAError(cudaMalloc((void **)&d_states_, np_*sizeof(curandState)));
    setup_kernel<<<np_/NUM_THREADS, NUM_THREADS>>>(d_states_);
    checkLastCUDAError("setup_kernel");
}

MCLGPU::~MCLGPU()
{
    printf("## MCLGPU destructor called!\n");
    checkCUDAError(cudaFree(d_particles_));
    checkCUDAError(cudaFree(d_weights_));
}

void MCLGPU::init_constants(
    float motion_dispersion_x, float motion_dispersion_y, float motion_dispersion_theta)
{
    printf("## MCLGPU::init_constants() called!\n");
    printf("     motion_dispersion_x:     %f\n", motion_dispersion_x);
    printf("     motion_dispersion_y:     %f\n", motion_dispersion_y);
    printf("     motion_dispersion_theta: %f\n", motion_dispersion_theta);
    checkCUDAError(cudaMemcpyToSymbol(c_motion_dispersion_x,
                                 &motion_dispersion_x, sizeof(c_motion_dispersion_x)));
    checkCUDAError(cudaMemcpyToSymbol(c_motion_dispersion_y,
                                 &motion_dispersion_y, sizeof(c_motion_dispersion_y)));
    checkCUDAError(cudaMemcpyToSymbol(c_motion_dispersion_theta,
                                 &motion_dispersion_theta, sizeof(c_motion_dispersion_theta)));
}

void MCLGPU::update(float *px, float *py, float *pangle,
                    float *odometry_delta,
                    double *weights)
{
    printf("## MCLGPU::update() called\n");
    /* Copy data from host to device */
    checkCUDAError(cudaMemcpy(d_particles_, px, sizeof(float)*np_, cudaMemcpyHostToDevice));
    checkCUDAError(cudaMemcpy(d_particles_+np_, py, sizeof(float)*np_, cudaMemcpyHostToDevice));
    checkCUDAError(cudaMemcpy(d_particles_+np_*2, pangle, sizeof(float)*np_, cudaMemcpyHostToDevice));

    float *d_odom_delta;
    checkCUDAError(cudaMalloc((void**)&d_odom_delta, sizeof(float)*3));
    checkCUDAError(cudaMemcpy(d_odom_delta, odometry_delta, sizeof(float)*3, cudaMemcpyHostToDevice));

    cuda_motion_sensor_model<<<np_/NUM_THREADS, NUM_THREADS>>>(
        d_states_, d_particles_, d_odom_delta,
        d_weights_,
        np_);

    checkLastCUDAError_noAbort("cuda_motion_sensor_model");

    checkCUDAError(cudaMemcpy(weights, d_weights_, sizeof(double)*np_, cudaMemcpyDeviceToHost));
}