#include "motor.h"

static const motor_params pmsms[] = {
    {
        .R = 2.5,
        .Ld = 67e-3,
        .Lq = 67e-3,
        .J = 5e-5,
        .P = 4,
        .B = 1,
        .lambda = 0.17,
    },
    {
        .R = 2.6,
        .Ld = 6.7e-3,
        .Lq = 6.7e-3,
        .J = 3.5e-5,
        .P = 3,
        .B = 5e-5,
        .lambda = 0.319,
    },
    {
        .R = 4.3,
        .Ld = 67e-3,
        .Lq = 27e-3,
        .J = 3.5e-5,
        .P = 4,
        .B = 5e-5,
        .lambda = 0.272,
    }
};

static const motor_params *pmsm = &pmsms[0];

int set_motor(unsigned int idx)
{
    if(idx < sizeof(pmsms)/sizeof(pmsms[0]))
    {
        pmsm = &pmsms[idx];
        return 0;
    }      
    return -1;
}

/*
   u[0] = id
   u[1] = iq
   u[2] = wr
   u[3] = vd
   u[4] = vq
   u[5] = Ti
   u[6] = theta
 */
void motor_eq(float t, float u[], float f[]) {

    f[ID] = -pmsm->R / pmsm->Ld * u[0] + pmsm->Lq / pmsm->Ld * u[2] * u[1] +
        1 / pmsm->Ld * u[3];
    f[IQ] = -pmsm->R / pmsm->Lq * u[1] - pmsm->Ld / pmsm->Lq * pmsm->P * u[2] * u[0] -
        (pmsm->lambda * pmsm->P * u[2]) / pmsm->Lq + 1 / pmsm->Lq * u[4];
    f[WR] = pmsm->P / pmsm->J *
        (pmsm->lambda * u[1] + (pmsm->Ld - pmsm->Lq) * u[1] * u[0]) -
        pmsm->B / pmsm->J * u[2] - u[5] / pmsm->J;
    f[VD] = 0;
    f[VQ] = 0;
    f[TI] = 0;
    f[THETA] = u[WR];
}

float get_torque(float *iq, float *id) {
    return 0.75 * pmsm->P * ((pmsm->Ld - pmsm->Lq) * (*id) * (*iq) + pmsm->lambda * (*iq));
}
