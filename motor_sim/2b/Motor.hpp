
// motsim.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#pragma once
#include <iostream>
#include <stdlib.h>
#include "StateMachine.hpp"
#include <math.h>
#define MAX_SPEED 5
/*
inputs:
    - currentU
    - currentV
    - currentW
    - speed
    - hallsensorU
    - hallsensorV
    - hallsensorW
    - diag
    - mot_temp
    - inverter_temp

output:
    - ul
    - uh
    - vh
    - vl
    - wh
    - wl
    - nBreak2
    - DRV_STRONG
    - LOWGAIN

use the maximum pwm frequency available
the more the frequency the less amps required to drive at specific speed

if the motor is not moving, disable it
*/

/*
Hallsensor
3 timer with xor  -> position
1 timer for calculating speed
*/
class HallSensor
{
public:
    static const float PI_180 = 3.14159265359 / 180;
    static constexpr uint16_t hall2angle[6][3] = {
        /*   uvw  l  h */
        {0b101, 0, 60 * PI_180},
        {0b100, 60 * PI_180, 120 * PI_180},
        {0b110, 120 * PI_180, 180 * PI_180},
        {0b010, 180 * PI_180, 240 * PI_180},
        {0b011, 240 * PI_180, 300 * PI_180},
        {0b001, 300 * PI_180, 360 * PI_180},
    };
    HallSensor()
    {
        /* probably set the timers and stuff */
    }
    void calculate_position()
    {
        /* gpio readpin, this should give me a 101 like input */
        /* dependency injection */
        m_position = hall2angle[hall_values][0];
    }

private:
    std::uint8_t hall_values; /* 000 0001 010 etc. */
    std::uint32_t counter;
    float m_position; /* U phase is zero degree */
    float m_speed;
};

class PID
{
    float m_P;
    float m_I;
    float m_D;

    float error;
    float prev;
    float integral;
    float output;

public:
    PID(float P, float I, float D) : m_P(P), m_I(I), m_D(D)
    {
        error = 0.0;
        integral = 0.0;
        prev = 0.0;
        output = 0.0;
    }

    float run(float &target, float &measurement)
    {
        error = target - measurement;
        output = m_P * error + m_I * integral + m_D * (error - prev);
        if (output > 1)
        {
            output = 1;
        }
        else if (output < -1)
        {
            output = -1;
        }
        if (integral < -1)
        {
            integral = -1;
        }
        if (integral > 1)
        {
            integral = 1;
        }
        prev = error;
        integral += error;
        return output;
    }
};
class Vector
{
private:
    float id;
    float iq;

public:

    Vector(float id, float iq):id(id), iq(iq) {};
    float u()
    {
        return id;
    }
    float q()
    {
        return iq;
    }
   
}; 

class SpeedController
{
private:
    float cur_speed;    /* actual speed reading from hall sensors / encoder */
    float target_speed; /* desired speed to user */
    float sp_speed;     /* setpoint for ramping( we cannot increase the speed at one timestep, motor goes F) */

    float speed_command;
    bool enable_ramp;
    float frequency; /* switching frequency */
    float incr; /* increment for ramp up/down */
    float id;
    float iq;
    float ud;
    float uq;
    float VDC;
    PID Speed_pid{2, 0.5, 0.05};
    PID ID_pid{3, 3, 4};
    PID IQ_pid{3, 3, 4};
    float kc[3][3];
public:
    SpeedController(float dc_voltage = 7.4, float incr = 0.1):VDC(dc_voltage), incr(incr)
    {
        cur_speed = 0.0;
        target_speed = 0.0;
        sp_speed = 0.0;
        speed_command = 0.0;
        enable_ramp = false;
        id = 0.0;
        iq = 0.0;
        ud = 0.0;
        uq = 0.0;
    }
    void set_speed(float new_speed)
    {
        target_speed = new_speed;
    }
    void loop_speed(float u[3], float position)
    {
        setkc(position);
        calcIdIq(u);
        run_speed();
    }
    void run_speed()
    {
        speed_command = Speed_pid.run(sp_speed, cur_speed);
    }
    void loop_current()
    {
        ud = ID_pid.run(id_command, id);
        uq = IQ_pid.run(0.0, iq); /* try to make it zero  */
        /* this gives like a voltage vector for me */
    }
    void calc_freq()
    {
        /* https://www.motioncontroltips.com/what-is-space-vector-pulse-width-modulation-svpwm/
         */
        float f_u, f_v, f_w;
        float S_u, S_v, S_w;
        int states[8];
        static constexpr float sectors[8][7] ={
         /* ud1 uq1 ud2 uq2 degree start, code1 code 2 */
           {0, 0, 0, 0, 0, 0, 0},
           {2/3, 0, 1/3, 1/ sqrt(3),0, 1, 3 },
           {1/3, 1/sqrt(3), -1/3, 1/sqrt(3), 60, 3, 2 },
           {-1/3, 1/sqrt(3), -2/3, 0, 120,2, 6 },
           {-2/3, 0, -1/3, -1/sqrt(3), 180, 6, 4 },
           {-1/3, -1/sqrt(3), 1/3, -1/sqrt(3), 240, 4, 5 },
           {1/3, -1/sqrt(3), 2/3, 0, 300, 5, 1 },
           {0, 0, 0, 0, 0, 7, 7 }
        };
        f_u = (2 * S_u - S_v - S_w) / 3.0;
        f_v = (2 * S_v - S_u - S_w) / 3.0;
        f_w = (2 * S_w - S_u - S_v) / 3.0;
        /* normalize */
        ud /= VDC;
        uq /= VDC;
        int sector_index = 0; /* degrees ? */
        /*  it should give the t1 t2 t0 t7 frequencies as follows
            t1 and t2 builds from the 6 vectors, vector math -> this will give the vector angle
            this vector size and the maximum voltage will be added by: (vdc + 0) / t_vdc = v_desired
            d vector is always the rotor state, so if someone want me to go to 3 halves, that means we are going fast,
            if i need to move for like 2 cm then it's not that big
        */
       for(int i = 0; i < 6; i++)
       {
           float ud_min;
           float ud_max;
           float uq_min;
           float uq_max;
           if(sectors[i][0] < sectors[i][2])
           {
               ud_min = sectors[i][0];
               ud_max = sectors[i][2];
           }
           else
           {
               ud_min = sectors[i][2];
               ud_max = sectors[i][0];
           }
           if(sectors[i][1] < sectors[i][3])
           {
               uq_min = sectors[i][1];
               uq_max = sectors[i][3];
           }
           else
           {
               uq_min = sectors[i][3];
               uq_max = sectors[i][1];
           }
           if(ud >= ud_min && ud <= ud_max && 
               uq >= uq_min && uq <= uq_max)
           {
               sector_index = i;
               break;
           }
       }
       /* build the vector */
       float td;
       float tq;
       calc_amount(sectors[sector_index][0] * VDC,
                   sectors[sector_index][2] * VDC,
                   sectors[sector_index][1] * VDC,
                   sectors[sector_index][3] * VDC,
                   td, tq);
        float t0 = 1 - td - tq; /* 0.2x first vector + 0.4x second vector, 0.4 is missing from whole */
        float t0 = 1.0/frequency;
        /* based on the two sectors  */
        float u_on_time = td * ((int)sectors[sector_index][5] & 1) + tq * ((int)sectors[sector_index][6] & 1) + 0.5 * t0 * 1;
        float v_on_time = td * ((int)sectors[sector_index][5]>>1 & 1) + tq * ((int)sectors[sector_index][6]>>1 & 1)+ 0.5 * t0 * 1;
        float w_on_time = td * ((int)sectors[sector_index][5]>>2 & 1) + tq * ((int)sectors[sector_index][6]>>2 & 1)+ 0.5 * t0 * 1;
        
    }

    /* 
        A: ud1 
        B: ud2 
        C: uq1
        D: uq2 
    */
    void calc_amount(float A, float B, float C, float D, float &x, float &y)
    {
        y = (ud- uq*C) / (D * A + C * B);
        x = (uq - y * D) / C;
    }
    static constexpr float SQRTFIDESZ = 0.81649658092;
    static const float PI = 3.14159265359;
    /*
    position in radian
    */
    void setkc(float position)
    {
        /* maybe an u - v */
        /* then the hall sensor position ? */
        /* last fix commutation + where are we at the current side */
        kc[0][0] = SQRTFIDESZ * cos(position);
        kc[0][1] = SQRTFIDESZ * cos(position - 2 * PI / 3);
        kc[0][2] = SQRTFIDESZ * cos(position + 2 * PI / 3);

        kc[1][0] = -SQRTFIDESZ * sin(position);
        kc[1][1] = -SQRTFIDESZ * sin(position - 2 * PI / 3);
        kc[1][2] = -SQRTFIDESZ * sin(position + 2 * PI / 3);

        kc[2][0] = 0.57735026919;
        kc[2][1] = 0.57735026919;
        kc[2][2] = 0.57735026919;
    }
    /*
    calculates the id iq currents from:
    u: current abc phase
    kc: park clark transform in one step need to be two phase to use cordic
    */
    void calcIdIq(float u[3])
    {
        id = u[0] * kc[0][0] + u[1] * kc[1][0] + u[2] * kc[2][0];
        iq = u[0] * kc[1][0] + u[1] * kc[1][1] + u[2] * kc[1][2];
    }
};

class Motor
{
public:
    enum ErrTypes
    {
        OVERCURRENT = 0,
        UNDERVOLTAGE,
        OVERTEMP_INV,
        OVERTEMP_MOT,
        ERROR_SIZE
    };
    enum State
    {
        STOP = 0, /* doing nothing with the motor */
        ERROR,    /* overcurrent, motor temp error, bus voltage error, inverter temp error */
        BREAKING, /* idk, break to zero maybe? */
        RUN,
        STATE_SIZE
    };
    enum Event
    {
        BREAK = 0,    /* break command issued from user, or for some reason we need to stop fast */
        CHANGE_SPEED, /* new speed command issued, we need to ramp up or down */
        ANY_BAD,      /* something went wrong, disable inverter, act accordingly extra monitoring etc.*/
        ALL_GOOD,     /* whatever went wrong, now everything is good, be happy */
        SPEED_REACHED,
        EVENT_SIZE
    };
    enum PhaseEnum
    {
        Ph_A = 0,
        Ph_B,
        Ph_C,
        PHASE_NUM
    };
    Motor()
    {
        /* automata init */
        incr = 0.2;          /* increase the motor speed 0.2 m/s in one timestep */
        state = State::STOP; /* standing still hopefully */
        enable_ramp = 0;
    }
    uint8_t ramp_up()
    {
        return target_speed > cur_speed;
    }
    void setState(State newState)
    {
        state = newState;
    }
    Motor::State getState()
    {
        return state;
    }

    void motor_task()
    {
        /* only ramp up if we reached the previous checkpoint and */
        if (enable_ramp)
        {
            if (ramp_up() && (cur_speed >= sp_speed))
            {
                if ((sp_speed + incr) <= target_speed)
                {
                    sp_speed += incr;
                }
                else
                {
                    sp_speed = target_speed;
                    /* this is more like ramping finsihed not speed reached but whatever, disable ramp up */
                }
            }
            /* ramp down */
            else if (cur_speed <= sp_speed)
            {
                if ((sp_speed - incr) >= target_speed)
                {
                    sp_speed -= incr;
                }
                else
                {
                    sp_speed = target_speed;
                }
            }
        }
        printf("Update pwm values to the gate based on speed = %f, id, iq \n", sp_speed);
        /* dummy update value */
        cur_speed = sp_speed;
    }
    /*input */
    std::uint16_t currentU;
    std::uint16_t currentV;
    std::uint16_t currentW;

    std::uint16_t speed; /*  input speed */

    std::uint16_t diag;

    std::uint16_t motor_temp;
    std::uint16_t inverter_temp;

    /* output */
    std::uint16_t uh;
    std::uint16_t ul;
    std::uint16_t vh;
    std::uint16_t vl;
    std::uint16_t wh;
    std::uint16_t wl;

    bool enable;
    bool nbreak1;
    bool nbreak2;

    bool drv_strong;
    bool lowgain;

    /* inner variables */

    State state;

    float mot_temp;
};
