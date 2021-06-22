#include "mcl_gpu.h"
#include "errorCheckingMacro.cuh"

#define DIST_THRESHOLD 0.0
#define STEP_COEFF 0.999

/* Constants stored in CUDA constant memory */
/* motion model dispersion constants */
__constant__ float c_motion_dispersion_x;
__constant__ float c_motion_dispersion_y;
__constant__ float c_motion_dispersion_theta;
/* downsampled angles */
__constant__ float c_angles[NUM_ANGLES];
/* Occupancy grid coordinates related constants */
__constant__ float c_world_origin_x;
__constant__ float c_world_origin_y;
__constant__ float c_world_angle;
__constant__ float c_world_sin_angle;
__constant__ float c_world_cos_angle;
__constant__ float c_world_scale;
__constant__ float c_inv_world_scale;
/* squash factor used for weight computation */
__constant__ double c_inv_squash_factor;


__device__ void print_constants()
{
    printf("@@ Variables in constant memory:\n");
    printf("   c_motion_dispersion_x:     %f\n", c_motion_dispersion_x);
    printf("   c_motion_dispersion_y:     %f\n", c_motion_dispersion_y);
    printf("   c_motion_dispersion_theta: %f\n", c_motion_dispersion_theta);
    printf("   c_angles (0-10): ");
    for (int i = 0; i < 10; i++) printf("%f  ", c_angles[i]); printf("\n");
    printf("   c_inv_squash_factor:       %f\n", c_inv_squash_factor);
    printf("   c_world_origin_x:          %f\n", c_world_origin_x);
    printf("   c_world_origin_y:          %f\n",  c_world_origin_y);
    printf("   c_world_angle:             %f\n",     c_world_angle);
    printf("   c_world_sin_angle:         %f\n", c_world_sin_angle);
    printf("   c_world_cos_angle:         %f\n", c_world_cos_angle);
    printf("   c_world_scale:             %f\n",     c_world_scale);
    printf("   c_inv_world_scale:         %f\n", c_inv_world_scale);
}

__device__ int clamp1(float val, float min, float max) {
	val = val>max?max:val;
	val = val<min?min:val;
	return (int)val;
}

__device__ float calc_range1(float x0, float y0, float theta, float *distMap, int width, int height, float max_range)
{
    float ray_direction_x = cosf(theta);
    float ray_direction_y = sinf(theta);

    int px = 0;
    int py = 0;

    float t = 0.0;
    float out = max_range;

    while (t < max_range) {
        px = x0 + ray_direction_x * t;
        py = y0 + ray_direction_y * t;

        if (px >= width || px < 0 || py < 0 || py >= height) {
            out = max_range;
            break;
        }

        float d = distMap[px * height + py]; //distance(px,py, distMap, width, height);

        if (d <= DIST_THRESHOLD) {
            float xd = px - x0;
            float yd = py - y0;
            out =  sqrtf(xd*xd + yd*yd);
            break;
        }

        t += fmaxf(d * STEP_COEFF, 1.0);
    }
    return out;
}

/* cuRAND state setup kernel */
__global__ void setup_kernel(curandState *state, int n)
{
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    if (id == 0) printf("@@ Kernel <<<setup_kernel>>> is called!\n");
    if (id >= n) return;
    /* Each thread gets same seed, a different sequence
       number, no offset */
    curand_init(1234, id, 0, &state[id]);
}

__global__ void cuda_motion_sensor_model(
    curandState *states,
    float *particles,
    float *odom_delta,
    int N,
    float *obs, /* observations, i.e. downsampled ranges from the lidar */
    double *sensorTable, int table_width, /* sensor table used by the sensor model */
    float *distMap, int width, int height, float max_range, /* distance map info */
    double *weights,
    /*
     * kernel launch parameter
     * blocks gives the total number of blocks launched
     * lastBlockNotFull tells whether the last block has inactive threads.
     * lastBlockNotFull == 0: no inactive threads
     * lastBlockNotFull > 0 : it gives the number of active threads in the last block
     */
    int blocks, int lastBlockNotFull)
{
    int p = threadIdx.x + blockIdx.x * blockDim.x;
    if (p == 0) {
        printf("@@ Kernel <<<cuda_sensor_model>>> is called! on %d threads per block\n", NUM_THREADS);
        print_constants();
    }
    if (p >= N) return;
    curandState localState = states[p];
    float x     = particles[p];
    float y     = particles[p + N];
    float angle = particles[p + N*2];

    /*****************************************
     * apply motion model to each particle
     *****************************************/
    float local_delta_x = cosf(angle) * odom_delta[0] - sinf(angle) * odom_delta[1];
    float local_delta_y = sinf(angle) * odom_delta[0] + cosf(angle) * odom_delta[1];
    float2 rnn2 = curand_normal2(&localState);
    float  rnn  = curand_normal(&localState);
    x += local_delta_x + rnn2.x * c_motion_dispersion_x;
    y += local_delta_y + rnn2.y * c_motion_dispersion_y;
    angle += rnn * c_motion_dispersion_theta;

    /************************************************
     * evaluate each particle with the sensor model
     ************************************************/
    /*
     * The shared memory is used as a local cache for a single row of the
     * sensor table. In each iteration of the sensor model loop, only one
     * row of the sensor table is accessed by all particles.
     */
    extern __shared__ double s_sensorTable_row[];
    /* convert the pose from world to grid coordinate */
    float x0 = (x - c_world_origin_x) * c_inv_world_scale;
    float y0 = (y - c_world_origin_y) * c_inv_world_scale;
    float temp = x0;
    x0 = c_world_cos_angle * x0 + c_world_sin_angle * y0;
    y0 = - c_world_sin_angle * temp + c_world_cos_angle * y0;

    double w = 1.0;
    for (int ai = 0; ai < NUM_ANGLES; ai ++)
    {
        /* convert the observation from world to grid coordinate */
        int r = clamp1(obs[ai] * c_inv_world_scale, 0, table_width - 1.0);
        /* load the r-th row of sensor table from global memory into shared memory */
        int idx = threadIdx.x;
        while (idx < table_width)
        {
            s_sensorTable_row[idx] = sensorTable[r*table_width + idx];
            /*
             * When the last block is not full, the step for threads in the last
             * block should be the number of active threads in the last block.
             */
            if (lastBlockNotFull && blockIdx.x == (blocks-1))
                idx += lastBlockNotFull;
            else
                idx += blockDim.x;
        }
        /* Compute the range of the given pose */
        float theta = angle - c_world_angle + c_angles[ai];
        float range = calc_range1(x0, y0, theta, distMap, width, height, max_range);
        int d = clamp1(range, 0, table_width - 1.0);
        /* Evaluate the current ray by questing the sensor table */
        __syncthreads();
        double eval = s_sensorTable_row[d];
        /* accumulate the weight */
        w *= eval;
        //printf("@@ %f: r=%d  d=%d  eval=%lf\n", theta, r, d, eval);

    }
    weights[p] = pow(w, c_inv_squash_factor);
    //printf("@@ w=%e  weight=%e\n", w, weights[p]);


    /* update particles with new pose */
    particles[p]     = x;
    particles[p+N]   = y;
    particles[p+N*2] = angle;
    states[p] = localState;
}

MCLGPU::MCLGPU(int num_particles): np_(num_particles)
{
    printf("## MCLGPU constructor called!! Number of particles: %d\n", np_);
    /* Allocated device space */
    checkCUDAError(cudaMalloc((void**)&d_particles_, sizeof(float)*np_*3));
    checkCUDAError(cudaMalloc((void**)&d_weights_, sizeof(double)*np_));

    /* Allocate device space for odometry_delta */
    checkCUDAError(cudaMalloc((void**)&d_odom_delta_, sizeof(float)*3));

    /*Allocate device space for obs */
    checkCUDAError(cudaMalloc((void**)&d_obs_, sizeof(float)*NUM_ANGLES));

    /* Initialize curand */
    checkCUDAError(cudaMalloc((void **)&d_states_, np_*sizeof(curandState)));
    int blocks = np_/NUM_THREADS;
    if (blocks == 0) blocks = 1;
    setup_kernel<<<blocks, NUM_THREADS>>>(d_states_, np_);
    checkLastCUDAError("setup_kernel");
}

MCLGPU::~MCLGPU()
{
    printf("## MCLGPU destructor called!\n");
    checkCUDAError(cudaFree(d_particles_));
    checkCUDAError(cudaFree(d_weights_));
}

void MCLGPU::set_angles(float *angles)
{
    printf("## MCLGPU::set_angles() called!\n");
    printf("   angles (0-10): ");
    for (int i = 0; i < 10; i++) printf("%f  ", angles[i]); printf("\n");
    checkCUDAError(cudaMemcpyToSymbol(c_angles, angles, sizeof(float)*NUM_ANGLES));
}

void MCLGPU::init_constants(
    float motion_dispersion_x,
    float motion_dispersion_y,
    float motion_dispersion_theta,
    double inv_squash_factor)
{
    printf("## MCLGPU::init_constants() called!\n");
    printf("     motion_dispersion_x:     %f\n", motion_dispersion_x);
    printf("     motion_dispersion_y:     %f\n", motion_dispersion_y);
    printf("     motion_dispersion_theta: %f\n", motion_dispersion_theta);
    printf("     inv_squash_factor:       %f\n", inv_squash_factor);
    checkCUDAError(cudaMemcpyToSymbol(c_motion_dispersion_x,
                                 &motion_dispersion_x, sizeof(float)));
    checkCUDAError(cudaMemcpyToSymbol(c_motion_dispersion_y,
                                 &motion_dispersion_y, sizeof(float)));
    checkCUDAError(cudaMemcpyToSymbol(c_motion_dispersion_theta,
                                 &motion_dispersion_theta, sizeof(float)));
    checkCUDAError(cudaMemcpyToSymbol(c_inv_squash_factor,
                                 &inv_squash_factor, sizeof(double)));
}

void MCLGPU::update(float *px, float *py, float *pangle,
                    float *odometry_delta, float *obs, int num_angles,
                    double *weights)
{
    printf("## MCLGPU::update() called\n");
    /* Copy particles from host to device */
    checkCUDAError(cudaMemcpy(d_particles_, px, sizeof(float)*np_, cudaMemcpyHostToDevice));
    checkCUDAError(cudaMemcpy(d_particles_+np_, py, sizeof(float)*np_, cudaMemcpyHostToDevice));
    checkCUDAError(cudaMemcpy(d_particles_+np_*2, pangle, sizeof(float)*np_, cudaMemcpyHostToDevice));

    /* copy odometry delta from host to device */
    checkCUDAError(cudaMemcpy(d_odom_delta_, odometry_delta, sizeof(float)*3, cudaMemcpyHostToDevice));
    /* copy observations from host to device */
    checkCUDAError(cudaMemcpy(d_obs_, obs, sizeof(float)*num_angles, cudaMemcpyHostToDevice));

    int blocks = np_/NUM_THREADS;
    int rem = np_ - NUM_THREADS * blocks;
    if (blocks == 0) blocks = 1;
    cuda_motion_sensor_model<<<blocks, NUM_THREADS, sizeof(double)*table_width_>>>(
        d_states_, d_particles_, d_odom_delta_,
        np_,
        d_obs_,
        d_sensorTable_, table_width_,
        d_distMap_, width_, height_, max_range_,
        d_weights_,
        blocks, rem);

    checkLastCUDAError_noAbort("cuda_motion_sensor_model");

    /* copy weights back to host */
    checkCUDAError(cudaMemcpy(weights, d_weights_, sizeof(double)*np_, cudaMemcpyDeviceToHost));
    /* copy particles back to host */
    checkCUDAError(cudaMemcpy(px, d_particles_, sizeof(float)*np_, cudaMemcpyDeviceToHost));
    checkCUDAError(cudaMemcpy(py, d_particles_+np_, sizeof(float)*np_, cudaMemcpyDeviceToHost));
    checkCUDAError(cudaMemcpy(pangle, d_particles_+np_*2, sizeof(float)*np_, cudaMemcpyDeviceToHost));
}

void MCLGPU::set_sensor_table(double *sensorTable, int t_w)
{
    printf("## set_sensor_table called!  table_width: %d\n", t_w);
    table_width_ = t_w;
    int table_size = sizeof(double) * table_width_ * table_width_;
    checkCUDAError(cudaMalloc((void**)&d_sensorTable_, table_size));
    checkCUDAError(cudaMemcpy(d_sensorTable_, sensorTable, table_size, cudaMemcpyHostToDevice));
}

void MCLGPU::set_map(ranges::OMap omap, float max_range)
{
    /* set map constants */
    checkCUDAError(cudaMemcpyToSymbol(c_world_origin_x,  &omap.world_origin_x, sizeof(float)));
    checkCUDAError(cudaMemcpyToSymbol(c_world_origin_y,  &omap.world_origin_y, sizeof(float)));
    checkCUDAError(cudaMemcpyToSymbol(c_world_angle,     &omap.world_angle, sizeof(float)));
    checkCUDAError(cudaMemcpyToSymbol(c_world_sin_angle, &omap.world_sin_angle, sizeof(float)));
    checkCUDAError(cudaMemcpyToSymbol(c_world_cos_angle, &omap.world_cos_angle, sizeof(float)));
    checkCUDAError(cudaMemcpyToSymbol(c_world_scale, &omap.world_scale, sizeof(float)));
    float inv_scale = 1.0 / omap.world_scale;
    checkCUDAError(cudaMemcpyToSymbol(c_inv_world_scale, &inv_scale, sizeof(float)));

    ranges::DistanceTransform *distImage = new ranges::DistanceTransform(&omap);
    width_ = distImage->width;
    height_ = distImage->height;
    max_range_ = max_range;
    float *raw_grid = new float[width_*height_];
    for (int i = 0; i < width_; ++i)
        std::copy(distImage->grid[i].begin(), distImage->grid[i].end(), &raw_grid[i*height_]);

    checkCUDAError(cudaMalloc((void**)&d_distMap_, width_*height_*sizeof(float)));
    checkCUDAError(cudaMemcpy(d_distMap_, raw_grid, width_*height_*sizeof(float), cudaMemcpyHostToDevice));
    free(raw_grid);
}