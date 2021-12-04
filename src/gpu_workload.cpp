#include <ros/ros.h>
#include <ros/console.h>

#include <cstdlib>

#include "gpu_workload.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mcl_gpu");

    ros::NodeHandle node;
    // ros::NodeHandle private_nh("~");

    //int load = strtol(argv[1], NULL, 10);
    // private_nh.getParam("load", load);

    GPU_Workload work(1792);

    //ROS_INFO("Dummy Kernel is running with load = %d", load);
    float acc_time = 0.0f;
    int cnt = 0;

    //ros::Rate r(100);
    while (ros::ok()) {
        auto t0 = ros::Time::now();
        work.callKernel();
        auto t1 = ros::Time::now();

        cnt ++;
        auto k_t = (t1 - t0).toSec()*1000.0;
        acc_time += k_t;
        if (cnt %10 == 0)
             printf("iter %3d: time %7.4f\n", cnt, acc_time / cnt);
        //if (cnt == 500) ros::shutdown();

        //r.sleep();
    }
    printf("%7.4f\n", acc_time / cnt);

}
