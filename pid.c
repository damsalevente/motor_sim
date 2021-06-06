#include "pid.h"

float pid(float *target, float *currval) {
  static float buffer = 0;
  float err;
  float answer;
  err = *target - *currval;
  buffer += err;
  answer = K * err + PID_I * buffer;
  answer = answer > 230 ? 230 : answer;
  answer = answer < 0 ? 0 : answer;
  return answer;
}