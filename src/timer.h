#ifndef TIMER_H_
#define TIMER_H_

#include <ros/ros.h>
#include <vector>
#include <chrono>

//using Clock = std::chrono::high_resolution_clock;
using Clock = ros::Time;

namespace Utils
{
    class CircularArray
    {
    public:
        CircularArray(int size);
        void append(float val);
        float mean();
    protected:
        std::vector<float> arr;
        int num_els;
        int ind;
    };



    class Timer
    {
    public:
        Timer(int smoothing);
        float tick();
        float fps();
    protected:
        CircularArray arr;
        Clock last_time;
    };
}


#endif
