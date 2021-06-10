#include "math.h"
#include "motor.h"
#include "controllers.h"
#include "pid.h"
#include "stdio.h"
#include "stdlib.h"
#define STEP_SIZE 1e-2
#define EPS 1e-3

float yt_a[N]; /* yt for adaptive rk4 */
float yt2[N];

typedef struct _rk_param {
    float yt[N]; /* yt for rk4 */
    float f1[N];
    float f2[N];
    float f3[N];
    float f4[N];
} rk_param;


static void rk4(float t, float ht, float y[], int n,
        void Func(float, float[], float[])) {
    static rk_param rk;
    float ht2, ht6;
    int i;

    ht2 = ht / 2e0;
    Func(t, y, rk.f1);
    for (i = 0; i < n; i++) {
        rk.yt[i] = y[i] + ht2 * rk.f1[i];
    }
    Func(t + ht2, rk.yt, rk.f2);
    for (i = 0; i <= 0; i++) {
        rk.yt[i] = y[i] + ht2 * rk.f2[i];
    }
    Func(t + ht, rk.yt, rk.f3);
    for (i = 0; i < n; i++) {
        rk.yt[i] = y[i] + ht * rk.f3[i];
    }
    Func(t + ht, rk.yt, rk.f4);
    ht6 = ht / 6e0;
    for (i = 0; i < n; i++) {
        y[i] += ht6 * (rk.f1[i] + 2 * (rk.f2[i] + rk.f3[i]) + rk.f4[i]);
    }
}

void adaptrk4(float t, float *ht, float *ht1, float eps, float y[], int n,
        void Func(float, float[], float[])) {

    float err, erri, f, ht2;
    int i, it;
    const int p = 4;
    static int itmax = 10;
    for (it = 0; it < itmax; it++) {
        ht2 = (*ht) / 2e0;
        for (i = 0; i < n; i++) {
            yt2[i] = y[i];
            yt_a[i] = y[i];
        }
        rk4(t, *ht, yt_a, n, Func);
        rk4(t, ht2, yt2, n, Func);
        rk4(t + ht2, ht2, yt2, n, Func);

        err = 0e0;
        for (i = 0; i < n; i++) {
            erri = yt2[i] ? fabs(1e0 - yt_a[i] / yt2[i]) : fabs(yt2[i] - yt_a[i]);
            if (err < erri) {
                err = erri;
            }
        }
        f = 1e0;
        if (err)
            f = 0.9e0 * pow(eps / err, 1e0 / p);
        if (f > 5e0)
            f = 5e0;
        *ht1 = f * (*ht);
        if (err <= eps)
            break;
        *ht = *ht1;
    }
    if (it > itmax)
        printf("Adapt max no of iterations exceeded! \n");
    for (i = 0; i < n; i++)
        y[i] = yt2[i];
}


void step(float t, float next_t,float u[])
{
    FILE *out = fopen("plot.csv", "a");
    float ht;
    float ht1 = STEP_SIZE;
    float torque;
    while(t + ht < next_t)
    {
        ht = ht1;
        adaptrk4(t, &ht, &ht1, EPS, u, N, motor_eq);
        torque = get_torque(&u[ID], &u[IQ]);
        t += ht;
    }
    fprintf(out, "%f,%f,%f,%f,%f,%f,%f,%f\n", t, u[0], u[1], u[2], u[3], u[4],
            u[5],u[THETA], torque);
    printf("Step:%lf\n\tI_d => %lf\t U_d => %lf\n\tI_q => %lf\tUq => %lf\n\tSpeed => %lf\n\tposition: %lf\n", t, u[0],u[VD], u[1],u[VQ], u[WR], u[THETA]);
    fclose(out);
}

void write_header()
{
    FILE *out = fopen("plot.csv", "w");
    fprintf(out, "t,id,iq,wr,ud,uq,tl,te,theta\n");
    fclose(out);
}

void motor_turn_on(float u[])
{
    for(int i = 0; i < N; i++)
    {
        u[i] = 0.0;
    }
}

int main() {
    float t,*u;
    float ud,uq;
    u = (float *)malloc(N * sizeof(float)); // solution components

    write_header();
    motor_turn_on(u);
    t = 0.0;
    for(int i = 0; i < 5000; i++)
    {
        control(30,u[WR],u[ID],u[IQ],&ud, &uq);
        u[VD] = ud;
        u[VQ] = uq;
        step(t, t + 0.1, u);
        t += 0.1;
    }

    free(u);
    return 0;
}
