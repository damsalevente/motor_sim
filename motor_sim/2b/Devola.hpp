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
           automata.entracte(&motor, Motor::CHANGE_SPEED, &speed);
           for(int i = 0; i < 10; i++)
           {
                motor.motor_task(); 
           }
           automata.entracte(&motor, Motor::Event::BREAK, &speed);
           automata.entracte(&motor, Motor::Event::BREAK, &speed);
           automata.entracte(&motor, Motor::Event::ANY_BAD, &speed);
           automata.entracte(&motor, Motor::Event::ALL_GOOD, &speed);
    }

private:
    Motor motor;
    StateMachine automata; /* rules */
};