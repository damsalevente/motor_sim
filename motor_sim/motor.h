#ifndef MOTOR_H
#define MOTOR_H
#define N 7 /* number of variables */

typedef enum params
{
    ID = 0,
    IQ,
    WR,
    VD,
    VQ,
    TI,
    THETA,
    ODE_COUNT
} params;
typedef struct motor_params
{
    float R;  /* resistance */
    float Ld; /* inductance for d reference frame */
    float Lq; /* inductance for q reference frame */
    float J;  /* TODO: what is it */
    float P;
    float B;
    float lambda;
} motor_params;

/* right hand side equation for 3 motor equations, id, iq, and wr */
void motor_eq(float t, float u[], float f[]);
/* calculate torque from id iq */
float get_torque(float *, float *);
/* set the motor parameters */
int set_motor(unsigned int idx);
#endif
