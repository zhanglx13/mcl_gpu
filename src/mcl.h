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
    std::string which_rm;
};



#endif
