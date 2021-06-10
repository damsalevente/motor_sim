#include "controllers.h"
#include "pid.h"
#include "stdio.h"
#include "math.h"
#include "motor.h"

static pi_struct controller_w = {.K = 1, .I=0, .lim_max = 50, .lim_min = -50};
static pi_struct controller_id = {.K = 0.5, .I=0.000001, .lim_max = 5000, .lim_min = -5000};
static pi_struct controller_iq = {.K = 1.5, .I=0.000001, .lim_max = 5000, .lim_min = -5000};

/* this is where i would use a neural net instead of tuning 3 pi controllers */
void control(float w_ref, float wr, float id,float iq, float *ud, float *uq)
{
    /* speed diff -> required torque */
    float is_ref = pi(&controller_w, &w_ref, &wr);
    printf("%lf\n", is_ref);

    /* current generator */
    float id_ref = 0;
    float iq_ref = is_ref; 

    /* current controllers */
    *ud = pi(&controller_iq, &id_ref, &id);
    *uq = pi(&controller_id, &iq_ref, &iq);
} 
