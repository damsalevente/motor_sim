typedef struct pi_struct
{
    float K;
    float I;
    float lim_max;
    float lim_min;
    float buffer;
} pi_struct;

/* no D rn */
float pi(pi_struct *dev, float *target, float *currval);
void change_params(pi_struct *dev, float k, float i);
