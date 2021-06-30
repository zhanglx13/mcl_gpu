#ifndef TIMER_H_
#define TIMER_H_

#include <ros/ros.h>
#include <vector>
#include <chrono>
#include <numeric>

//using Clock = std::chrono::high_resolution_clock;
using Clock = ros::Time;

namespace Utils
{
    template <class T>
    class CircularArray
    {
    public:
        CircularArray(int size);
        void append(T val);
        T mean();
    protected:
        std::vector<T> arr;
        int num_els;
        int ind;
    };

    template <class T>
    CircularArray<T>::CircularArray(int size)
    {
        arr.resize(size);
        ind = 0;
        num_els = 0;
    }

    template <class T>
    void CircularArray<T>::append(T val)
    {
        if (num_els < (int)arr.size())
            num_els ++;
        arr[ind] = val;
        ind = (ind + 1) % arr.size();
    }

    template <class T>
    T CircularArray<T>::mean()
    {
        return std::accumulate(arr.begin(), arr.begin()+num_els, 0.0) / num_els;
    }


    class Timer
    {
    public:
        Timer(int smoothing);
        float tick();
        float fps();
    protected:
        CircularArray<float> arr;
        Clock last_time;
    };
}


#endif
