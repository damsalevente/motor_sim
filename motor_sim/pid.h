typedef struct pi_struct
{
    float K;
    float I;
    float D;
    float res;
    float err;
    float lim_max;
    float lim_min;
    float buffer;
    float prev_e;
} pi_struct;

/* no D rn */
float pi(pi_struct *dev, float *target, float *currval);
void change_params(pi_struct *dev, float k, float i, float d);
