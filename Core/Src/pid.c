/*
 * pid.c
 *
 *  Created on: Jan 3, 2026
 *      Author: Franciszek Rzesny
 */

#include "pid.h"
#include <math.h>


void PID_init(PID_t *par, float kp, float ki, float kd)
{
    par->integral = 0.0f;
    par->prev_input = 0.0f;

    par->kp = kp;
    par->ki = ki;
    par->kd = kd;

    par->anti_windup = 10;
    par->out_max = 100;
    par->out_min = -100;
}

float PID_update(PID_t *pid, float input, float dt)
{
    float error = pid->setpoint - input;

    // --- Integral (anti-windup)
    if (fabsf(error) < pid->anti_windup) {
        pid->integral += error * dt;
    }

    if (fabsf(pid->integral) > 50)
    {
    	pid->integral = 0;
    }

    // --- Derivative
    float derivative = (input - pid->prev_input) / dt;

    float output =
          pid->kp * error
        + pid->ki * pid->integral
        - pid->kd * derivative;

    // --- Saturacja
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    pid->prev_input = input;

    return output;
}
