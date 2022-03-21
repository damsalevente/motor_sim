#pragma once

#include <stdlib.h>
#include <iostream>
#include <cstddef>

#include "Motor.hpp"

class Transition
{
public:
    Transition()
    {
        m_next = Motor::State::STOP;
        cbk = NULL;
    }
    /* maybe this is only state dependent function so i only need one fucntion for every state*/
    void set(Motor::State next, void (*callback)(Motor *, uint16_t *))
    {
        m_next = next;
        cbk = callback;
    }
    Motor::State get(Motor *m, uint16_t *data)
    {
        if(cbk == NULL)
        {
            return Motor::State::STATE_SIZE;
        }
        cbk(m, data);
        return m_next;
    }

private:
    Motor::State m_next; /* maximum 255 state */
    void (*cbk)(Motor *, uint16_t *);
};

class StateMachine
{
public:
    StateMachine()
    {
        automata[Motor::STOP][Motor::CHANGE_SPEED].set(Motor::State::RUN, on_update_speed);
        automata[Motor::RUN][Motor::BREAK].set(Motor::State::BREAKING, on_break);
        automata[Motor::RUN][Motor::SPEED_REACHED].set(Motor::State::RUN, on_stop_ramp);
        automata[Motor::BREAKING][Motor::SPEED_REACHED].set(Motor::State::STOP, nothing);
        automata[Motor::BREAKING][Motor::CHANGE_SPEED].set(Motor::RUN, on_update_speed);
        automata[Motor::BREAKING][Motor::ANY_BAD].set(Motor::ERROR, on_error);
        automata[Motor::RUN][Motor::ANY_BAD].set(Motor::ERROR, on_error);
        automata[Motor::ERROR][Motor::ALL_GOOD].set(Motor::STOP, nothing);
    }
    static void on_update_speed(Motor *m, uint16_t * data)
    {
        printf("set speed\n");
        m->sc.set_speed(*data);    
    }
    static void on_break(Motor *m, uint16_t * data)
    {
        printf("on break");
    }
    static void on_stop_ramp(Motor *m, uint16_t * data)
    {
        printf("stop ramp\n");
    }
    static void nothing(Motor *m, uint16_t * data)
    {
        printf("nothing \n");
    }

    static void on_error(Motor *m, uint16_t *data)
    {
        printf("error \n");
    }

    const void entracte(Motor *m, Motor::Event e, uint16_t *data)
    {
        Motor::State s =automata[m->getState()][e].get(m, data); /* get the current (state - event) transition, run the handler, set the new state */
        if(s == Motor::State::STATE_SIZE)
        {
            return;
        }
        m->setState(s);
    }

private:
    Transition automata[Motor::State::STATE_SIZE][Motor::Event::EVENT_SIZE];
};
