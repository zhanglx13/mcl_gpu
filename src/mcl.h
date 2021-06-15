#ifndef MCL_H_
#define MCL_H_

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

    void update();
    /* difference implementations of MCL algorithm */
    void MCL_cpu();
    void MCL_gpu();
    void MCL_adaptive();

    void motion_model();
    void sensor_model();
    void print_particles(int n);

protected:
    /* parameters */
    int p_angle_step_;
    int p_max_particles_;
    int p_max_viz_particles_;
    float p_inv_squash_factor_;
    float p_max_range_meters_;
    int p_theta_discretization_;
    std::string p_which_rm_;
    std::string p_which_impl_;
    int p_publish_odom_;
    int p_do_viz_;

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

    /* internal state used by the MCL algorithm */
    bool lidar_initialized_;
    /*
     * 0: not receive any odom message yet
     * 1: received the first odom message
     * 2: odometry_delta_ is initialized
     */
    bool odom_initialized_;
    bool map_initialized_;

    /*
     * Note that this is different from geometry_msgs::Pose in the way that
     * last_pose_[2] is the angle but geometry_msgs::Pose[2] is queternion.
     */
    std::array<float, 3> last_pose_;
    std::array<float, 3> inferred_pose_;
    /*
     * This is the difference between the newly received and the last odometry
     * in the car frame. There needs to be a conversion between the odom frame
     * to the car frame.
     * Note that the rotation is expressed as angle instead of quaternion
     */
    std::array<float, 3> odometry_delta_ {0};
    ros::Time last_stamp_;


    /* Information about the map */
    int map_width_; // deprecated
    int map_height_; // deprecated
    ranges::OMap omap_;
    int p_max_range_px_;
    char *permissible_region_; // deprecated, do not use
    /*
     * free_cell_id_ store (x,y) of free cells of the map
     * Note that x corresponds to the column and y corresponds to the row.
     */
    std::vector<std::array<int,2>> free_cell_id_;

    /* particles */
    std::vector<double> weights_;
    float * particles_;
    std::vector<float> particles_x_;
    std::vector<float> particles_y_;
    std::vector<float> particles_angle_;

    /* containers for range data */
    std::vector<float> downsampled_ranges_;
    std::vector<float> downsampled_angles_;
    std::vector<float> viz_queries_;
    std::vector<float> viz_ranges_;

};

namespace utils
{
    std::array<float, 3> map_to_world(std::array<float, 3> p_in_map, ranges::OMap omap);
    void print_particles(std::vector<float> &x, std::vector<float> y, std::vector<float> angle, std::vector<double> weights);
}

#endif
