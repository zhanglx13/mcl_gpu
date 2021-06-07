#ifndef MCL_H_
#define MCL_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


class MCL
{
public:
    MCL();
    ~MCL();

    void scanCallback(const sensor_msgs::LaserScan& scan);
    void get_omap();

protected:
    /* parameters */
    std::string p_which_rm_;
    std::string p_scan_topic_;

    /* sensor model constants */
    float p_z_short_;
    float p_z_max_;
    float p_z_rand_;
    float p_z_hit_;
    float p_sigma_hit_;

    /* node handler */
    ros::NodeHandle node_;

    ros::Subscriber scan_sub_;
};



#endif
