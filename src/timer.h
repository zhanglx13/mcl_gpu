#ifndef TIMER_H_
#define TIMER_H_

#include <vector>
#include <chrono>

using Clock = std::chrono::high_resolution_clock;

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
        void tick();
        float fps();
    protected:
        CircularArray arr;
        Clock::time_point last_time;
    };
}


#endif
