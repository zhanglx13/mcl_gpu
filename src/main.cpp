#include <ros/ros.h>

#include "mcl.h"
#include "nav_msgs/GetMap.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mcl_gpu");


    ros::NodeHandle node;
    ros::NodeHandle private_nh;
    float max_range;
    private_nh.param("max_range", max_range, 30.0f);
    ROS_INFO("Getting OMap object from map server!!");
    /* get OccupancyGrid map from the map_server's /static_map service */
    ros::ServiceClient map_service_client = node.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap srv_map;
    if (map_service_client.waitForExistence())
    {
        if (map_service_client.call(srv_map))
        {
            ROS_INFO("Getting map from service: %s", map_service_client.getService().c_str());
            const nav_msgs::OccupancyGrid& map (srv_map.response.map);
            ROS_INFO("    resolution: %f", map.info.resolution);
            ROS_INFO("    width:      %d", map.info.width);
            ROS_INFO("    height:     %d", map.info.height);
            //p_max_range_px_ = static_cast<int>(p_max_range_meters_ / map.info.resolution);
            ranges::OMap omap = ranges::OMap(map);
            //ROS_INFO("Initialize range method");
            //range_method = ranges::RayMarchingGPU(omap, p_max_range_meters_);
            MCL mcl(omap, max_range);
            ros::spin();
        }
        else
        {
            ROS_ERROR("Failed to call map service");
            return 0;
        }
    }



    return(0);
}
