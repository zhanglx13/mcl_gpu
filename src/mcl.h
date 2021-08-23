#ifndef MCL_H_
#define MCL_H_

#include <mutex>
#include <condition_variable>
#include <thread>
#include <tuple>
#include <unordered_map>
#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PolygonStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

#include "range_libc/RangeLib.h"
#include "mcl_gpu.h"
#include "resampling_gpu.h"
#include "timer.h"



class MCL
{
public:
    MCL(ranges::OMap omap, float max_range);
    ~MCL();

    /* This is the only interface that is public to client */
    void update();

private:
    /* callback functions */
    void lidarCB(const sensor_msgs::LaserScan& msg);
    void odomCB(const nav_msgs::Odometry& msg);
    void pose_initCB(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void rand_initCB(const geometry_msgs::PointStamped& msg);

    void get_omap();
    void precompute_sensor_model();
    void initialize_global();
    void initialize_initpose(int startover);
    void initialize_acc();
    void initialize_mt();
    void initialize_workers(int num_threads);
    void do_acc(float time_in_ms);
    double calc_diff(pose_t);
    float calc_dis(pose_t);

    /* different implementations of MCL algorithm */
    std::tuple<float, float, float> MCL_cpu();
    std::tuple<float, float, float> MCL_gpu();
    std::tuple<float, float, float> MCL_hybrid();
    void gpu_update(int N_gpu);
    void cpu_update();
    void t_cpu_update(int start, int num_particles, int num_threads);
    void MCL_adaptive();
    int particle_partition();

    void motion_model(int start, int num_particles);
    void sensor_model(int start, int num_particles);
    void resampling();
    double normalize_weight();
    void resampling_gpu();
    ResamplingGPU *resgpu_;

    void expected_pose();
    void publish_tf();

    void visualize();
    void publish_particles(fvec_t px, fvec_t py, fvec_t pangle, int num_particles);
    void select_particles(fvec_t &px, fvec_t &py, fvec_t &pangle, int num_particles);

    /* For debugging and testing */
    void print_particles(int n);
    void calc_range_one_pose(pose_t ins, fvec_t &ranges, bool printonly);
    void set_fake_angles_and_ranges(int pidx);

    /* parameters */
    int p_angle_step_;
    int p_max_particles_;
    int p_N_gpu_;
    int p_max_viz_particles_;
    int p_cpu_threads_;
    double p_inv_squash_factor_;
    float p_max_range_meters_;
    int p_theta_discretization_;
    std::string p_which_rm_;
    std::string p_which_impl_;
    std::string p_which_viz_;
    std::string p_which_expect_;
    std::string p_which_res_;
    int p_publish_odom_;
    int p_do_viz_;
    float p_init_var_;

    /* sensor model constants */
    float p_z_short_;
    float p_z_max_;
    float p_z_rand_;
    float p_z_hit_;
    float p_sigma_hit_;

    /* bacauese RayMarchingGPU::set_sensor_model expects table to be of type double */
    double *sensor_model_table_;

    /* motion model constants */
    float p_motion_dispersion_x_;
    float p_motion_dispersion_y_;
    float p_motion_dispersion_theta_;

    std::string p_scan_topic_;
    std::string p_odom_topic_;

    /* node handler */
    ros::NodeHandle node_;

    /* Publishers for visualizations */
    ros::Publisher pose_pub_;
    ros::Publisher particle_pub_;
    ros::Publisher fake_scan_pub_;
    ros::Publisher rect_pub_;
    ros::Publisher odom_pub_;

    /* publisher for coordinates */
    tf::TransformBroadcaster tf_pub_;

    /* These topics are to receive data from the racecar */
    ros::Subscriber scan_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber click_sub_;

    ranges::RayMarchingGPU rmgpu_;
    ranges::RayMarching rm_;
    MCLGPU *mclgpu_;

    /* internal state used by the MCL algorithm */
    int lidar_initialized_{0};
    /*
     * 0: not receive any odom message yet
     * 1: received the first odom message
     * 2: odometry_delta_ is initialized
     */
    int odom_initialized_{0};
    int map_initialized_{0};

    /*
     * Note that this is different from geometry_msgs::Pose in the way that
     * last_pose_[2] is the angle but geometry_msgs::Pose[2] is queternion.
     */
    pose_t last_pose_;
    pose_t inferred_pose_;
    pose_t init_pose_;
    int init_pose_set_{0};
    /*
     * This is the difference between the newly received and the last odometry
     * in the car frame. There needs to be a conversion between the odom frame
     * to the car frame.
     * Note that the rotation is expressed as angle instead of quaternion
     */
    pose_t odometry_delta_ {0};
    ros::Time last_stamp_;


    /* Information about the map */
    int map_width_; // deprecated
    int map_height_; // deprecated
    ranges::OMap omap_;
    int p_max_range_px_;
    // char *permissible_region_; // deprecated, do not use
    /*
     * free_cell_id_ store (x,y) of free cells of the map
     * Note that x corresponds to the column and y corresponds to the row.
     */
    std::vector<std::array<int,2>> free_cell_id_;

    /* particles */
    std::vector<double> weights_;
    float * particles_;
    fvec_t particles_x_;
    fvec_t particles_y_;
    fvec_t particles_angle_;
    /* temp particles used by resampling */
    fvec_t px_;
    fvec_t py_;
    fvec_t pangle_;

    /* containers for range data */
    fvec_t downsampled_ranges_;
    fvec_t downsampled_angles_;
    fvec_t viz_queries_;
    fvec_t viz_ranges_;

    /* mutex to protect shared data between threads */
    std::mutex odom_mtx_;
    std::mutex range_mtx_;

    /*
     * @note
     *
     * Multi-threaded synchronization using condition variable
     *
     * cpu_workers_ is an array of thread handles. We allocate PCORES threads
     * since it is expected to yield the best performance. Note that when GPU
     * implementation is involved, only PCORES-1 threads are used to implement
     * the CPU version of MCL, since one thread is dedicated to launch the GPU
     * kernels.
     *
     * cv_ is a condition variable and ready_ is a flag.
     * They are used to synchronize between the main thread
     * and the worker threads. Here is how we are going to use it:
     *   1. In MCL_cpu() or MCL_hybrid(), the main thread set item_ to be the
     *      number of workers and ready_ to be 1. And it signal_all() on cv_.
     *      Therefore, all worker threads can start update the particles.
     *   2. After the main thread finishes its portion, it waits on item_
     *      to be 0. If item_ is not 0, it wait() on cv_, expecting the last
     *      worker to signal it.
     *   3. When a worker thread starts, it checks ready_ to see if ready_ is
     *      set to one. If not, it wait() on cv_, expecting the main thread
     *      will signal it.
     *   4. When a worker thread finishes, it decrement item_ and checks if
     *      item_ is 0. If so, it signal() on cv_ and wakes up the main thread.
     * Details can be found in the comment before cpu_update() function.
     *
     * Some notes
     * - Worker threads are guaranteed to start after the main thread has set
     *   ready_ and item_ and signal_all()
     * - Only the last thread will signal() the main thread. It is possible that
     *   it also wakes up other threads (since other threads have finished and
     *   started to wait on ready_ to be larger than 0 again for the next iteration).
     *   However, at this point, ready_ is 0, so only the main thread will wake up
     *   and continue its work. Other worker threads will wake up and go back to
     *   sleep immediately.
     * - It is also possible that the main thread has not finished its portion
     *   before the last worker thread sends the signal. In this case, the signal
     *   is lost. Fortunately, item_ is set to 0 and main thread will not need
     *   to wait() on cv_. Therefore, it can continue its work directly.
     *
     * ready_mtx_ is used to protect concurrent access to ready_ and item_ from
     * all threads.
     *
     * worker_start_map_ and worker_n_map_ are used to represent the working set for
     * each thread, i.e. the start index and the total number of particles.
     */
    std::vector<std::thread> cpu_workers_;
    std::condition_variable cv_;
    int ready_ {0};
    int item_ {0};
    std::mutex cv_mtx_;
    std::unordered_map<pthread_t, int> worker_start_map_;
    std::unordered_map<pthread_t, int> worker_n_map_;

    /* flag to control whether to perform resampling */
    int do_res_{0};

    /* info of each iteration of the MCL algorithm */
    int iter_ {0};
    int focus_iter_ {0};
    Utils::Timer timer_;
    Utils::CircularArray<double> maxW_;
    Utils::CircularArray<double> diffW_;
    Utils::CircularArray<double> dis_;
    float acc_error_x_ {0};
    float acc_error_y_ {0};
    float acc_error_angle_ {0};
    float acc_time_ms_ {0};
    float acc_focus_time_ms_ {0};
    float acc_res_ {0};
    float acc_update_ {0};
    float acc_total_ {0};
    float acc_expect_ {0};
};

template <class T>
class PlusWithNoise
{
public:
    PlusWithNoise(unsigned seed, T stddev) :
        seed_(seed), stddev_(stddev),
        dist_ (std::normal_distribution<T>(0.0, stddev_)),
        gen_ (std::default_random_engine(seed_)) {}
    T operator()(T a, T b) {return a + b + dist_(gen_); }
private:
    unsigned seed_;
    T stddev_;
    std::default_random_engine gen_;
    std::normal_distribution<T> dist_;
};

#endif
