#include "main.h"
#include "controllers.h"
#include "motor.h"
#include "pid.h"
#include "solver.h"
#include "stdlib.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

#define TS 0.05 /* time step for simulation */

int main(int argc, char *argv[])
{
    /* motor parameters */
    float t;  /* time */
    int counter = 0;
    int ctrl_run = 0;
    float u[N] = {0.0};   /* buffer for output */
    int duration;
    float w_ref = 0.0;
    write_header();
    if(argc < 3)
    {
        printf("ERROR: give me how much time do you want to run the simulator, and what is the target speed\n defaulting to 1 sec and w_ref = 1 rad/s\n");
        duration = 1;
        w_ref = 1.0;

    }
    else
    {
        duration = atoi(argv[1]);
        w_ref = atoi(argv[2]);
    }
    set_motor(0);
    motor_turn_on(u);
    t = 0; /* reset motor sim to zero */
    /* wait for connection */
    while (t < duration)
    {
        /* run motor model */
        step(t, t + TS, u);
        /* run the controller */
        control_runner(&w_ref, &u[WR], &u[ID], &u[IQ], &u[VD], &u[VQ]); 
        if(t > duration / 2 )
        {
            w_ref = -w_ref;
        }
        t += TS;
    }
    return 0;
}

