/*
 * pid.h
 *
 *  Created on: Jan 3, 2026
 *      Author: franu
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_input;

    float out_min;
    float out_max;

    float setpoint;
    float anti_windup;

} PID_t;


void PID_init(PID_t *par, float kp, float ki, float kd);
float PID_update(PID_t *pid, float input, float dt);

#endif /* INC_PID_H_ */
