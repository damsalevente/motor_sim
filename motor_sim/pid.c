#include "pid.h"

float pi(pi_struct *dev, float *target, float *currval)
{
    dev->err = (*target) - (*currval);
    dev->buffer += dev->err;
    
    dev->res = dev->K * dev->err + dev->I * dev->buffer + dev->D * (dev->err-dev->prev_e);

    dev->res = dev->res > dev->lim_max ? dev->lim_max : dev->res;
    dev->res = dev->res < dev->lim_min ? dev->lim_min : dev->res;
    return dev->res;
}

void change_params(pi_struct *dev, float k, float i, float d)
{
   dev->K = k;
   dev->I = i;
   dev->D = d;
}
