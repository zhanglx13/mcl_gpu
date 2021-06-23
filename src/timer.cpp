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

void Utils::Timer::tick()
{
    Clock::time_point t = Clock::now();
    std::chrono::duration<float, std::milli> elapsed_ms = t - last_time;
    arr.append(elapsed_ms.count());
    last_time = t;
}

float Utils::Timer::fps()
{
    return arr.mean();
}
