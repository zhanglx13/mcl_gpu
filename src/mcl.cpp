#include "mcl.h"

MCL::MCL()
{
    ros::NodeHandle private_nh_("~");
    private_nh_.param("which_rm", p_which_rm_, std::string("rmgpu"));
    private_nh_.param("scan_topic", p_scan_topic_, std::string("scan"));

    private_nh_.param("z_short", p_z_short_, 0.01f);
    private_nh_.param("z_max", p_z_max_, 0.07f);
    private_nh_.param("z_rand", p_z_rand_, 0.12f);
    private_nh_.param("z_hit", p_z_hit_, 0.75f);
    private_nh_.param("sigma_hit", p_sigma_hit_, 8.0f);

    // std::cout<< "range method: " << which_rm << std::endl;
    ROS_INFO("Parameters: ");
    ROS_INFO("  Range method: %s", p_which_rm_.c_str());
    ROS_INFO("  Scan topic: %s", p_scan_topic_.c_str());

    scan_sub_ = node_.subscribe(p_scan_topic_, 1, &MCL::scanCallback, this);

    /* initialize the state */
    get_omap();
}

MCL::~MCL()
{
    // std::cout<< "All done, bye you!!"<< std::endl;
    ROS_INFO("All done, bye yo!!");
}

void MCL::scanCallback(const sensor_msgs::LaserScan& scan)
{
    float angle_min = scan.angle_min;
    float angle_max = scan.angle_max;
    float num_rays = (angle_max - angle_min)/scan.angle_increment;
    int ahead_idx = static_cast<int>(num_rays/2.0);
    ROS_INFO("Received scan data, range ahead is %f",scan.ranges[ahead_idx]);
}

void MCL::get_omap()
{
    ROS_INFO("Getting OMap objest from map server!!");
}
