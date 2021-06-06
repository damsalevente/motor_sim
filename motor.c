#include "motor.h"

static motor_params pmsm = {
    .R = 4.3,
    .Ld = 27e-4,
    .Lq = 63e-4,
    .J = 5e-5,
    .P = 3,
    .B = 1,
    .lambda = 0.22,
};

/* 
u[0] = id
u[1] = iq
u[2] = wr
u[3] = vd
u[4] = vq
u[5] = Ti
*/
void motor_eq(float t, float u[], float f[]) {

  f[0] = -pmsm.R / pmsm.Ld * u[0] + pmsm.Ld / pmsm.Lq * pmsm.P * u[2] * u[1] +
         1 / pmsm.Ld * u[3];
  f[1] = -pmsm.R / pmsm.Lq * u[1] - pmsm.Ld / pmsm.Lq * pmsm.P * u[2] * u[0] -
         (pmsm.lambda * pmsm.P * u[2]) / pmsm.Lq + 1 / pmsm.Lq * u[4];
  f[2] = pmsm.P / pmsm.J *
             (pmsm.lambda * u[1] + (pmsm.Ld - pmsm.Lq) * u[1] * u[0]) -
         pmsm.B / pmsm.J * u[2] - u[5] / pmsm.J;
}

float get_torque(float *iq, float *id)
{
    return 3 * pmsm.P/4 * ((pmsm.Ld - pmsm.Lq) * (*id)* (*iq) + pmsm.lambda * (*iq));
}
