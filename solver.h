#include "motor.h"
#define STEP_SIZE 1e-2
#define EPS 1e-3

typedef struct _rk_param
{
    float yt[N]; /* yt for rk4 */
    float f1[N];
    float f2[N];
    float f3[N];
    float f4[N];
} rk_param;

void step(float t, float next_t, float u[]);
void motor_turn_on(float u[]);
void write_header();