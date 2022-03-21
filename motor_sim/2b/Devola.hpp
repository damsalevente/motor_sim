#pragma once
#include <iostream>
#include <cstddef>
#include "StateMachine.hpp"
#include "Motor.hpp"

class Devola
{
public:
    Devola()
    {
    }
    Devola(Motor m) : motor(m)
    {
    }
    void march(uint16_t speed)
    {
        float m_currs[3] = {motor.currentU, motor.currentV, motor.currentW};
        motor.sc.input(0, m_currs);
        automata.entracte(&motor, Motor::CHANGE_SPEED, &speed);
        for (int i = 0; i < 10; i++)
        {
            float position = 0.0;
            float phase_currents[3] = {motor.currentU, motor.currentV, motor.currentW};
            motor.sc.input(position, phase_currents);
            if (i % 4 == 0)
            {
                motor.sc.loop();
            }
            motor.sc.loop_current();
            if (motor.sc.speed_reached())
            {
                automata.entracte(&motor, Motor::Event::SPEED_REACHED, 0);
            }
        }
        automata.entracte(&motor, Motor::Event::BREAK, &speed);
        automata.entracte(&motor, Motor::Event::BREAK, &speed);
        automata.entracte(&motor, Motor::Event::ANY_BAD, &speed);
        automata.entracte(&motor, Motor::Event::ALL_GOOD, &speed);
    }
    /* print every info available */
    void report()
    {
        /* todo reporter/logger class with options to where when what etc. */
        /* every element has it's own reporter (dclink reporter, phase reporter) */
        motor.log();
        motor.sc.log();
    }

private:
    Motor motor;
    StateMachine automata; /* rules */
};