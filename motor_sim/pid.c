#include "pid.h"

float pi(pi_struct *dev, float *target, float *currval)
{
    float err;
    float answer;
    err = (*target) - (*currval);
    dev->buffer += err;
    
    answer = dev->K * err + dev->I * dev->buffer;

    answer = answer > dev->lim_max ? dev->lim_max : answer;
    answer = answer < dev->lim_min ? dev->lim_min : answer;
    return answer;
}

void change_params(pi_struct *dev, float k, float i)
{
   dev->K = k;
   dev->I = i;
}
