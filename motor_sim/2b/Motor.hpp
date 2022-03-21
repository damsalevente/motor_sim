
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
    static constexpr float PI_180 = 3.14159265359 / 180;
    static constexpr float hall2angle[6][3] = {
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

    float run(float target, float &measurement)
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
    Vector(float id, float iq) : id(id), iq(iq){};
    float u()
    {
        return id;
    }
    float q()
    {
        return iq;
    }
};

typedef struct
{
    float ud;
    float uq;
    float ud60; /* ud value for the next sector (60 degrees diference) */
    float uq60;
    float angle; /* start angle in degrees  */
    int s;       /* which bridge(s) should be open */
    int s60;     /* which bridge(s) should be open */
} Sector;

class SpeedController
{
private:
    float cur_speed;    /* actual speed reading from hall sensors / encoder */
    float target_speed; /* desired speed to user */
    float sp_speed;     /* setpoint for ramping( we cannot increase the speed at one timestep, motor goes F) */
    float speed_command;
    bool enable_ramp; /* whether to increase/decrease the internal setpoint */
    float frequency; /* switching frequency */
    float incr;      /* increment for ramp up/down */

    float id;        /* measured d reference current */
    float iq;
    
    float ud; /* calculated d reference command from PID controller */
    float uq;

    float VDC; /* dc voltage that is attached to it */

    float kc[3][3];

    float pwm_u;
    float pwm_v;
    float pwm_w;

    const Sector sectors[8] = {
        /* ud1 uq1 ud2 uq2 degree start, code1 code 2 */
        {0, 0, 0, 0, 0, 0, 0},
        {2 / 3, 0, 1 / 3, 1 / sqrt(3), 0, 1, 3},
        {1 / 3, 1 / sqrt(3), -1 / 3, 1 / sqrt(3), 60, 3, 2},
        {-1 / 3, 1 / sqrt(3), -2 / 3, 0, 120, 2, 6},
        {-2 / 3, 0, -1 / 3, -1 / sqrt(3), 180, 6, 4},
        {-1 / 3, -1 / sqrt(3), 1 / 3, -1 / sqrt(3), 240, 4, 5},
        {1 / 3, -1 / sqrt(3), 2 / 3, 0, 300, 5, 1},
        {0, 0, 0, 0, 0, 7, 7}};
    PID Speed_pid{2, 0.5, 0.05};
    PID ID_pid{3, 3, 4};
    PID IQ_pid{3, 3, 4};

public:
    SpeedController(float dc_voltage = 7.4, float incr = 0.1) : VDC(dc_voltage), incr(incr)
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
        if(target_speed == new_speed)
        {
            return;
        }
        target_speed = new_speed;
        enable_ramp = true;
    }
    void loop()
    {
        if(enable_ramp)
        {
            if(ramp_up())
            {
                sp_speed += incr;
                if(sp_speed >= target_speed)
                {
                    sp_speed = target_speed;
                    enable_ramp = false;
                }
            }
            else
            {
                sp_speed -= incr;
                if(sp_speed <= target_speed)
                {
                    sp_speed = target_speed;
                    enable_ramp = false;
                }
            }
        }
        speed_update();
    }
    void speed_update()
    {
        speed_command = Speed_pid.run(sp_speed, cur_speed);
    }
    bool speed_reached()
    {
        return cur_speed == target_speed;
    }
    void input(float position, float phase_currents[3])
    {
        set_Kc(position);
        calcIdIq(phase_currents);
    }

    bool ramp_up()
    {
        return target_speed < cur_speed;
    }

    /* low ms */
    void loop_current()
    {
        ud = ID_pid.run(speed_command, id);
        uq = IQ_pid.run(0.0, iq); /* try to make it zero  */
        update_pwm();
    }
    void update_pwm()
    {
        int sector_index = 0; /* select one plane from the hexagon */
        /* scale vectors to -1 1 */
        ud /= VDC;
        ud = ud > 1 ? 1 : ud;
        ud = ud < -1 ? -1 : ud;

        uq /= VDC;
        uq = uq > 1 ? 1 : uq;
        uq = uq < -1 ? -1 : uq;

        sector_index = get_sector_index();
        /* build the pwm signal times */
        calc_pwm_signals(sectors[sector_index]);
    }

    void calc_pwm_signals(const Sector &s)
    {
        float u_on_time, v_on_time, w_on_time;
        float y = (ud - uq * s.uq) / (s.uq60 * s.ud + s.uq * s.ud60); /* 60 degree vector amount */
        float x = (uq - y * s.uq60) / s.uq;                           /* 0 degree vector amount */
        float t0 = 1 - x - y;                                         /* 0.2x first vector + 0.4x second vector, 0.4 is missing from whole */

        t0 *= frequency;
        x *= frequency;
        y *= frequency;

        u_on_time = x * (s.s & 1) + y * (s.s60 & 1) + 0.5 * t0;
        v_on_time = x * (s.s >> 1 & 1) + y * (s.s60 >> 1 & 1) + 0.5 * t0;
        w_on_time = x * (s.s >> 2 & 1) + y * (s.s60 >> 2 & 1) + 0.5 * t0;

        /* TODO: check error */
        pwm_u = u_on_time;
        pwm_v = v_on_time;
        pwm_w = w_on_time;
    }
    void log()
    {
        printf("Speedcontroller: Input:\n %lf %lf %lf\n ", pwm_u, pwm_v, pwm_w);
    }
    int get_sector_index()
    {
        int sector_index = 0;
        if (uq > 0)
        {
            if (ud > 0)
            {
                if (uq - 3 * ud < 0)
                {
                    sector_index = 1;
                }
                else
                {
                    sector_index = 2;
                }
            }
            else
            {
                if (uq + 3 * ud < 0)
                {
                    sector_index = 3;
                }
                else
                {
                    sector_index = 2;
                }
            }
        }
        else
        {
            if (ud > 0)
            {
                if (uq + 3 * ud < 0)
                {
                    sector_index = 5;
                }
                else
                {
                    sector_index = 6;
                }
            }
            else
            {
                if (uq - 3 * ud < 0)
                {
                    sector_index = 5;
                }
                else
                {
                    sector_index = 4;
                }
            }
        }
        return sector_index;
    }
    /*
    position in radian
    */
    void set_Kc(float position)
    {
        /* maybe an u - v */
        /* then the hall sensor position ? */
        /* last fix commutation + where are we at the current side */
        kc[0][0] = sqrt(2 / 3) * cos(position);
        kc[0][1] = sqrt(2 / 3) * cos(position - 2 * M_PI / 3);
        kc[0][2] = sqrt(2 / 3) * cos(position + 2 * M_PI / 3);

        kc[1][0] = -sqrt(2 / 3) * sin(position);
        kc[1][1] = -sqrt(2 / 3) * sin(position - 2 * M_PI / 3);
        kc[1][2] = -sqrt(2 / 3) * sin(position + 2 * M_PI / 3);

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
        state = State::STOP; /* standing still hopefully */
    }
    Motor(SpeedController sc):sc(sc)
    {
        /* automata init */
        state = State::STOP; /* standing still hopefully */
    }
    void setState(State newState)
    {
        state = newState;
    }
    Motor::State getState()
    {
        return state;
    }
    void log()
    {
        printf("Currents(u,v,w): %d %d %d \n", currentU, currentV, currentW);
        printf("Speed: % d\n", speed);
    }
    void motor_task()
    {
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
    SpeedController sc;
    State state;
    float mot_temp;
};
