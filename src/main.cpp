#include <ros/ros.h>
#include <ros/console.h>
#include <thread>

#include "mcl.h"
#include "nav_msgs/GetMap.h"

void spin_thread()
{
    ros::spin();
}

int main(int argc, char** argv)
{
    const pid_t pid = getpid();
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    const int set_result = sched_setaffinity(pid, sizeof(cpu_set_t), &cpuset);
    if (set_result != 0) {
        ROS_ERROR("sched_setaffinity: %d", set_result);
    }

    std::cout << "main thread "<< pthread_self() << " on cpu " << sched_getcpu() << "\n";

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::init(argc, argv, "mcl_gpu");


    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");
    int NP;
    private_nh.getParam("max_particles", NP);
    if ( ( NP > NUM_THREADS ) && ( NP % NUM_THREADS != 0 ) ){
        ROS_ERROR("max_particles must be a multiple of %d", NUM_THREADS);
        ros::shutdown();
    }
    float max_range;
    /*
      @note
     * max_range is the maximum range in meters that Lidar can measure.
     * max_range_px is the maximum range in the map, which is calculated as
     * max_range/omap.world_scale
     * max_range_px + 1 will be used as the width of the sensor table.
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
              @note
             * For the occupancy grid map, width and height are the number of cells in x and y dimension
             * resolution is meters per cell
             * origin is the pose of cell[0][0] in world frame. Sometimes the map can be rotated.
             */
            ranges::OMap omap = ranges::OMap(map);
#ifdef PRINT_INFO
            ROS_INFO("    resolution: %f", map.info.resolution);
            ROS_INFO("    width:      %d", map.info.width);
            ROS_INFO("    height:     %d", map.info.height);
            ROS_INFO("    origin (pose of cell[0][0])");
            ROS_INFO("       x: %f", map.info.origin.position.x);
            ROS_INFO("       y: %f", map.info.origin.position.y);
            ROS_INFO("       angle: %f", omap.quaternion_to_angle(map.info.origin.orientation));
            ROS_INFO("       quat: [%f %f %f %f]",
                     map.info.origin.orientation.x,
                     map.info.origin.orientation.y,
                     map.info.origin.orientation.z,
                     map.info.origin.orientation.w
                );
            /*
             * HNAME, PCORES, and LCORES are figured out and defined by CMake
             */
            std::cout<< "hostname: " << HNAME << "\n";
            printf("physical cores: %d\n", PCORES);
            printf("logical cores: %d\n", LCORES);
#endif

            /*
              @note
             * The second argument (max_range/omap.world_scale) passed to the
             * constructor of MCL will be used to initialize an object of
             * RayMarchingGPU. It is supposed to be the max range in the map.
             */
            MCL mcl(omap, max_range/omap.world_scale);

            /* spawn another thread to handle callbacks */
            std::thread t_spin(spin_thread);

            ros::Rate r(100); // 100 Hz
            while (ros::ok())
            {
                mcl.update();
                r.sleep();
            }
            if (t_spin.joinable())
                t_spin.join();
        }
        else
        {
            ROS_ERROR("Failed to call map service");
            return 0;
        }
    }



    return(0);
}
