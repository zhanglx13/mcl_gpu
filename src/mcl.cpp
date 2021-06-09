#include "mcl.h"

//#include <nav_msgs/OccupancyGrid.h>
//#include "nav_msgs/GetMap.h"


MCL::MCL(ranges::OMap omap, float max_range): rmgpu_ (omap, max_range)
{
    ros::NodeHandle private_nh_("~");
    private_nh_.param("angle_step", p_angle_step_);
    private_nh_.param("max_particles", p_max_particles_);
    private_nh_.param("max_viz_particles", p_max_viz_particles_);
    float tmp;
    private_nh_.param("squash_factor", tmp);
    p_inv_squash_factor_ = 1.0f/tmp;
    p_max_range_meters_ = max_range;
    private_nh_.param("theta_discretization", p_theta_discretization_);
    private_nh_.param("which_rm", p_which_rm_, std::string("rmgpu"));
    private_nh_.param("publish_odom", p_publish_odom_, 1);
    private_nh_.param("viz", p_do_viz_);

    private_nh_.param("scan_topic", p_scan_topic_, std::string("scan"));
    //private_nh_.param("max_range", p_max_range_meters_, 30.0f);

    private_nh_.param("z_short", p_z_short_, 0.01f);
    private_nh_.param("z_max", p_z_max_, 0.07f);
    private_nh_.param("z_rand", p_z_rand_, 0.12f);
    private_nh_.param("z_hit", p_z_hit_, 0.75f);
    private_nh_.param("sigma_hit", p_sigma_hit_, 8.0f);

    p_max_range_px_ = static_cast<int> (max_range / omap.world_scale);

    // std::cout<< "range method: " << which_rm << std::endl;
    ROS_INFO("Parameters: ");
    ROS_INFO("    Angle step:    %d", p_angle_step_);
    ROS_INFO("    Max particles: %d", p_max_particles_);
    ROS_INFO("    Max viz particles: %d", p_max_viz_particles_);
    ROS_INFO("    Inv squash factor: %f", p_inv_squash_factor_);
    ROS_INFO("    Max range meters:  %f", p_max_range_meters_);
    ROS_INFO("    Theta discretization:  %d", p_theta_discretization_);
    ROS_INFO("    Range method:  %s", p_which_rm_.c_str());
    ROS_INFO("    Publish odom:  %d", p_publish_odom_);
    ROS_INFO("    Do viz:  %d", p_do_viz_);

    ROS_INFO("    Scan topic:    %s", p_scan_topic_.c_str());
    ROS_INFO("    max_range_px:  %d", p_max_range_px_);

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
    // ROS_INFO("Getting OMap object from map server!!");
    // /* get OccupancyGrid map from the map_server's /static_map service */
    // ros::ServiceClient map_service_client = node_.serviceClient<nav_msgs::GetMap>("static_map");
    // nav_msgs::GetMap srv_map;
    // if (map_service_client.waitForExistence())
    // {
    //     if (map_service_client.call(srv_map))
    //     {
    //         ROS_INFO("Getting map from service: %s", map_service_client.getService().c_str());
    //         const nav_msgs::OccupancyGrid& map (srv_map.response.map);
    //         ROS_INFO("    resolution: %f", map.info.resolution);
    //         ROS_INFO("    width:      %d", map.info.width);
    //         ROS_INFO("    height:     %d", map.info.height);
    //         p_max_range_px_ = static_cast<int>(p_max_range_meters_ / map.info.resolution);
    //         ranges::OMap omap = ranges::OMap(map);
    //         //ROS_INFO("Initialize range method");
    //         //range_method = ranges::RayMarchingGPU(omap, p_max_range_meters_);
    //     }
    //     else
    //     {
    //         ROS_ERROR("Failed to call map service");
    //         return ;
    //     }
    // }
}
