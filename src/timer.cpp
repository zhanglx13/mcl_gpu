#include "timer.h"

Utils::Timer::Timer(int smoothing): arr(Utils::CircularArray<float>(smoothing))
{
    last_time = Clock::now();
}

float Utils::Timer::tick()
{
    Clock t = Clock::now();
    ros::Duration elapsed_ms = t - last_time;
    arr.append(elapsed_ms.toSec()*1000.0);
    last_time = t;
    return elapsed_ms.toSec()*1000.0;
}

float Utils::Timer::fps()
{
    return arr.mean();
}
