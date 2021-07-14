#ifndef MCL_H_
#define MCL_H_

#include <mutex>
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
#include "timer.h"



class MCL
{
public:
    MCL(ranges::OMap omap, float max_range);
    ~MCL();

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
    void do_acc(float time_in_ms);
    double calc_diff(pose_t);
    float calc_dis(pose_t);

    void update();
    /* different implementations of MCL algorithm */
    double MCL_cpu();
    double MCL_gpu();
    double MCL_hybrid();
    void t_gpu_update(int N_gpu);
    void MCL_adaptive();
    int particle_partition();

    void motion_model(int start, int num_particles);
    void sensor_model(int start, int num_particles);
    void resampling();

    void expected_pose();
    void publish_tf();

    void visualize();
    void publish_particles(fvec_t px, fvec_t py, fvec_t pangle, int num_particles);
    void select_particles(fvec_t &px, fvec_t &py, fvec_t &pangle, int num_particles);

    /* For debugging and testing */
    void print_particles(int n);
    void calc_range_one_pose(pose_t ins, fvec_t &ranges, bool printonly);
    void set_fake_angles_and_ranges(int pidx);

protected:
    /* parameters */
    int p_angle_step_;
    int p_max_particles_;
    int p_N_gpu_;
    int p_max_viz_particles_;
    double p_inv_squash_factor_;
    float p_max_range_meters_;
    int p_theta_discretization_;
    std::string p_which_rm_;
    std::string p_which_impl_;
    std::string p_which_viz_;
    std::string p_which_expect_;
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
    int lidar_initialized_;
    /*
     * 0: not receive any odom message yet
     * 1: received the first odom message
     * 2: odometry_delta_ is initialized
     */
    int odom_initialized_;
    int map_initialized_;

    /*
     * Note that this is different from geometry_msgs::Pose in the way that
     * last_pose_[2] is the angle but geometry_msgs::Pose[2] is queternion.
     */
    pose_t last_pose_;
    pose_t inferred_pose_;
    pose_t init_pose_;
    int init_pose_set_;
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

    /* containers for range data */
    fvec_t downsampled_ranges_;
    fvec_t downsampled_angles_;
    fvec_t viz_queries_;
    fvec_t viz_ranges_;

    /* mutex to protect shared data between threads */
    std::mutex odom_mtx_;
    std::mutex range_mtx_;

    /* flag to control whether to perform resampling */
    int do_res_;

    /* info of each iteration of the MCL algorithm */
    int iter_;
    Utils::Timer timer_;
    Utils::CircularArray<double> maxW_;
    Utils::CircularArray<double> diffW_;
    Utils::CircularArray<double> dis_;
    float acc_error_x_;
    float acc_error_y_;
    float acc_error_angle_;
    float acc_time_ms_;
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
