#include "timer.h"
#include <numeric>

Utils::CircularArray::CircularArray(int size)
{
    arr.resize(size);
    ind = 0;
    num_els = 0;
}

void Utils::CircularArray::append(float val)
{
    if (num_els < (int)arr.size())
        num_els ++;
    arr[ind] = val;
    ind = (ind + 1) % arr.size();
}

float Utils::CircularArray::mean()
{
    return std::accumulate(arr.begin(), arr.begin()+num_els, 0.0) / num_els;
}

Utils::Timer::Timer(int smoothing): arr(Utils::CircularArray(smoothing))
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
