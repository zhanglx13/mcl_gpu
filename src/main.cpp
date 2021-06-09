#include <ros/ros.h>

#include "mcl.h"
#include "nav_msgs/GetMap.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mcl_gpu");


    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");
    float max_range;
    /*
     * max_range is the maximum range that Lidar can measure. This is used to compute the
     * width of the sensor_model_table. We only need to compute the sensor model for r and
     * d that are within the max_range/resolution, which is p_max_range_px.
     */
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
            /*
             * For the occupancy grid map, width and height are the number of cells in x and y dimension
             * resolution is meters per cell
             * origin is the pose of cell[0][0] in world frame. Sometimes the map can be rotated.
             */
            ROS_INFO("    resolution: %f", map.info.resolution);
            ROS_INFO("    width:      %d", map.info.width);
            ROS_INFO("    height:     %d", map.info.height);
            ranges::OMap omap = ranges::OMap(map);

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
