#include "controllers.h"
#include "math.h"
#include "motor.h"
#include "pid.h"
#include "stdio.h"
#define COUNT_MAX 5

int count = 0;
float command_signal = 0.0;

static pi_struct controller_w = {
    .K = 0.05, .I = 0.01, .lim_max = 50, .lim_min = -50};
static pi_struct controller_id = {
    .K = 2.5, .I = 0.01, .lim_max = 300, .lim_min = -300, .buffer = 0.0};
static pi_struct controller_iq = {
    .K = 2.5, .I = 0.01, .lim_max = 300, .lim_min = -300, .buffer = 0.0};

void control_runner(float *w_ref, float *wr, float *id, float *iq, float *ud,
                    float *uq) {
  float id_ref;
  float iq_ref;

  if (count == 0)
  {
    control_slow(w_ref, wr, &command_signal);
  }
  id_ref = 0;
  iq_ref = command_signal - 1;

  count++;
  if (count > COUNT_MAX) {
    count = 0;
  }
  control_fast(&id_ref, &iq_ref, id, iq, ud, uq);
  printf("Ud: %lf, Uq: %lf\n",*ud, *uq );
}

/* id iq controller runs way more */
void control_fast(float *id_ref, float *iq_ref, float *id_act, float *iq_act,
                  float *ud, float *uq) {
  *ud = pi(&controller_id, id_ref, id_act);
  *uq = pi(&controller_iq, iq_ref, iq_act);
}
/* speed controller runs slower */
void control_slow(float *w_ref, float *w, float *command_signal) {
  /* this should convert speed to desired torque */
  *command_signal = pi(&controller_w, w_ref, w);
}
