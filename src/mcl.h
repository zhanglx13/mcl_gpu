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

protected:
    /* parameters */
    std::string p_which_rm_;
    std::string p_scan_topic_;
    /* node handler */

    ros::NodeHandle node_;

    ros::Subscriber scan_sub_;
};



#endif
