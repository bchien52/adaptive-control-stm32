/**
 * pid_controller.c
 *
 * Thu vien PID Controller
 * Created on: May 13, 2026
 * Author: Dang Bao Chien
 *
 */
 
#include "pi_controller.h"

static float clamp(float val, float lo, float hi)
{
    if (val > hi) return hi;
    if (val < lo) return lo;
    return val;
}

void pid_init(PID_t *pid, float kp, float ki, float ts)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->ts = ts;

    pid->integral_max = PID_INTEGRAL_MAX;   
    pid->integral_min = PID_INTEGRAL_MIN;  
    pid->output_max   = PID_OUTPUT_MAX;
    pid->output_min   = PID_OUTPUT_MIN;

    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
}

float pid_compute(PID_t *pid, float setpoint, float feedback)
{
    // Buoc 1: Tinh sai so
    float error = setpoint - feedback;

    // Buoc 2: Cap nhat tich phan - I(k) = I(k-1) + e(k) * Ts
    pid->integral += error * pid->ts;

    // Kep tich phan trong [integral_min .. integral_max]
    pid->integral = clamp(pid->integral, pid->integral_min, pid->integral_max);

    // Buoc 3: Tinh ngo ra u(k) = Kp*e(k) + Ki*I(k)
    pid->output = (pid->kp * error) + (pid->ki * pid->integral);

    // Kep ngo ra trong [output_min .. output_max]
    pid->output = clamp(pid->output, pid->output_min, pid->output_max);

    // Luu sai so cho vong tiep theo
    pid->prev_error = error;

    return pid->output;
}

void pid_reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}


void pid_set_integral_limit(PID_t *pid, float max, float min)
{
    pid->integral_max = max;
    pid->integral_min = min;
    pid->integral = clamp(pid->integral, pid->integral_min, pid->integral_max);
}

void pid_set_output_limit(PID_t *pid, float max, float min)
{
    pid->output_max = max;
    pid->output_min = min;
}
