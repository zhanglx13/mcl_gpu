#include <ros/ros.h>

#include "mcl.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mcl_gpu");

    MCL mcl;

    ros::spin();

    return(0);
}
