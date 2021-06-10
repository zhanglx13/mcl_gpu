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

    void get_omap(ranges::OMap omap);
    void precompute_sensor_model();
    void initialize_global(ranges::OMap omap);

protected:
    /* parameters */
    int p_angle_step_;
    int p_max_particles_;
    int p_max_viz_particles_;
    float p_inv_squash_factor_;
    float p_max_range_meters_;
    int p_theta_discretization_;
    std::string p_which_rm_;
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
    int p_max_range_px_;

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
    bool odom_initialized_;
    bool map_initialized_;

    /* Information about the map */
    int map_width_;
    int map_height_;
    char *permissible_region_; // deprecated, do not use
    /*
     * free_cell_id_ store (x,y) of free cells of the map
     * Note that x corresponds to the column and y corresponds to the row.
     */
    std::vector<std::array<int,2>> free_cell_id_;

    /* particles */
    double * weights;
    float * particles;
};

namespace utils
{
    std::array<float, 3> map_to_world(std::array<float, 3> p_in_map, ranges::OMap omap);
}

#endif
