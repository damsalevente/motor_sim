#include "math.h"
#include "motor.h"
#include "pid.h"
#include "stdio.h"
#include "stdlib.h"
#define N 6

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

int main() {
  float eps, ht, ht1, t, t1, t2, tmax, T0, T, u0, du0, us, *u;
  float torque = 0;
  float target_speed = 0;
  float buffer = 0;
  int n, nT, stepcount;
  FILE *out = fopen("plot.csv", "w");
  fprintf(out, "t,id,iq,wr,ud,uq,tl,te\n");
  stepcount = 0;
  tmax = 8;     // time span
  ht = 0.001e0; // time step size
  ht1 = ht;     // output
  eps = 1e-3;

  u = (float *)malloc(N * sizeof(float)); // solution components
  t = 0e0;
  u[0] = 0;
  u[1] = 0;
  u[2] = 0;
  u[3] = 0;
  u[4] = 0;
  u[TI] = 0;
  fprintf(out, "%f,%f,%f,%f,%f,%f,%f,%f\n", t, u[0], u[1], u[2], u[3], u[4],
          u[5]);
  while (t + ht < tmax) {
    stepcount++;
    ht = ht1;
    adaptrk4(t, &ht, &ht1, eps, u, N, motor_eq);
    torque = get_torque(&u[ID], &u[IQ]);
    // rk4(t,ht,u,N,Pendulum);
    if (t > 2) {
      target_speed = 5;
      //u[VD] = 230;
      //u[VQ] = 230;
    }
    u[3] = pid(&target_speed, &u[WR]);
    u[4] = u[3];

    t += ht;
    fprintf(out, "%f,%f,%f,%f,%f,%f,%f,%f\n", t, u[0], u[1], u[2], u[3], u[4],
            u[5], torque);
    printf("Step:%lf\n\tu0 => %lf\n\tu1 => %lf, Speed => %lf\n", t, u[0], u[1],
           u[WR]);
  }
  free(u);
  fclose(out);
  printf("Stepcount: %d\n", stepcount);
  return 0;
}
