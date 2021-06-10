#include "controllers.h"
#include "motor.h"
#include "pid.h"
#include "solver.h"
#include "stdlib.h"
#include <stdio.h>

int main() {
  float t, *u;
  float ud, uq, ref_speed;
  u = (float *)malloc(N * sizeof(float)); // solution components

  write_header();
  motor_turn_on(u);
  t = 0.0;
  for (int i = 0; i < 5000; i++) {
    if (i < 100) {
      ref_speed = 0;
    }
    if (i > 200) {
      ref_speed = 4;
    }

    if (i > 500) {
      ref_speed = 500;
    }
    control(ref_speed, u[WR], u[ID], u[IQ], &ud, &uq);
    step(t, t + 0.1, u);
    t += 0.1;
  }

  free(u);
  return 0;
}
