#ifndef MCL_H_
#define MCL_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "range_libc/RangeLib.h"


class MCL
{
public:
    MCL(ranges::OMap omap, float max_range);
    ~MCL();

    void scanCallback(const sensor_msgs::LaserScan& scan);
    void get_omap(ranges::OMap omap);
    void precompute_sensor_model();
    void initialize_global();

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
    int p_max_range_px_;



    /* node handler */
    ros::NodeHandle node_;

    ros::Subscriber scan_sub_;

    ranges::RayMarchingGPU rmgpu_;

    /* internal state used by the MCL algorithm */
    bool lidar_initialized_;
    bool odom_initialized_;
    bool map_initialized_;

    /* Information about the map */
    int map_width_;
    int map_height_;
    char *permissible_region_;
};



#endif
