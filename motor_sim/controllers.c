#include "controllers.h"
#include "math.h"
#include "motor.h"
#include "pid.h"
#include "stdio.h"
#define COUNT_MAX 5

static pi_struct controller_w = {.K = 50, .I = 0.05, .lim_max = 50, .lim_min = -50};
static pi_struct controller_id = {
    .K = 1, .I = 0.00001, .lim_max = 5000, .lim_min = -5000};
static pi_struct controller_iq = {
    .K = 10, .I = 0.001, .lim_max = 5000, .lim_min = -5000};


void control_runner(float *w_ref, float *wr, float *id, float *iq, float *ud, float *uq)
{
    static int count = 0;
    static float command_signal = 0.0;
    static float id_ref = 0.0;
    static float iq_ref = 0.0;

    if(count == 0)
    {
        control_slow(w_ref, wr, &command_signal);
    }
    id_ref = 0;
    iq_ref = 50;
    control_fast(&id_ref, &iq_ref, id, iq, ud, uq);

    count++;
    if(count = COUNT_MAX)
    {
        count = 0;
    }

}

/* id iq controller runs way more */
void control_fast(float *id_ref, float *iq_ref, float *id_act, float *iq_act, float *ud, float *uq)
{
    *ud = pi(&controller_id, id_ref, id_act);
    *uq = pi(&controller_iq, iq_ref, iq_act);
    printf("%lf\n %lf\n", *id_act, *iq_act);
}
/* speed controller runs slower */
void control_slow(float *w_ref, float *w, float *command_signal)
{
    /* this should convert speed to desired torque */
    *command_signal = pi(&controller_w, w_ref, w);
}
