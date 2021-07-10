#include "pid.h"

float pi(pi_struct *dev, float *target, float *currval)
{
    static float buffer = 0;
    float err;
    float answer;
    err = (*target) - (*currval);
    buffer += err;
    answer = dev->K * err + dev->I * buffer;
    answer = answer > dev->lim_max ? dev->lim_max : answer;
    answer = answer < dev->lim_min ? dev->lim_min : answer;
    return answer;
}
