#include "mcl.h"

MCL::MCL()
{
    ros::NodeHandle private_nh_("~");
    private_nh_.param("which_rm", p_which_rm_, std::string("rmgpu"));
    private_nh_.param("scan_topic", p_scan_topic_, std::string("scan"));

    // std::cout<< "range method: " << which_rm << std::endl;
    ROS_INFO("Range method: %s", p_which_rm_.c_str());
    ROS_INFO("Scan topic: %s", p_scan_topic_.c_str());

    scan_sub_ = node_.subscribe(p_scan_topic_, 1, &MCL::scanCallback, this);
}

MCL::~MCL()
{
    // std::cout<< "All done, bye you!!"<< std::endl;
    ROS_INFO("All done, bye you");
}

void MCL::scanCallback(const sensor_msgs::LaserScan& scan)
{
    ROS_INFO("Received scan data, max range is %f",scan.range_max);
}
