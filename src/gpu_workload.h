#ifndef _GPU_WORKLOAD_H_
#define _GPU_WORKLOAD_H_

class GPU_Workload
{
public:
    GPU_Workload(int sz);
    ~GPU_Workload();
    void callKernel();
private:
    int *d_input, *d_output;
    int *input, *output;
    int sz_;
};

#endif
