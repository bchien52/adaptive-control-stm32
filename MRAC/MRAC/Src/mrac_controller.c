/**
 * mrac_controller.c
 *
 * Thu vien MRAC Controller - MIT Rule
 * Created on: May 13, 2026
 * Author: Dang Bao Chien
 *
 */

#include "mrac_controller.h"

static float clamp(float val, float lo, float hi)
{
    if (val > hi)
        return hi;
    if (val < lo)
        return lo;
    return val;
}

void mrac_init(MRAC_t *mrac, float kp_init, float ki_init, float gamma_p, float gamma_i, float am, float ts)
{
    // Tham so co dinh
    mrac->ts = ts;
    mrac->gamma_p = gamma_p;
    mrac->gamma_i = gamma_i;
    mrac->am = am;

    // Kp, Ki khoi dau tu gia tri PI da biet on dinh
    mrac->kp = kp_init;
    mrac->ki = ki_init;

    // Gioi han mac dinh
    mrac->kp_min = MRAC_KP_MIN;
    mrac->kp_max = MRAC_KP_MAX;
    mrac->ki_min = MRAC_KI_MIN;
    mrac->ki_max = MRAC_KI_MAX;
    mrac->integral_min = MRAC_INTEGRAL_MIN;
    mrac->integral_max = MRAC_INTEGRAL_MAX;
    mrac->output_min = MRAC_OUTPUT_MIN;
    mrac->output_max = MRAC_OUTPUT_MAX;

    // Xoa trang thai
    mrac_reset(mrac);
}

float mrac_compute(MRAC_t *mrac, float setpoint, float feedback)
{
	  // Buoc 1: Cap nhat mo hinh mau
    float am_ts = mrac->am * mrac->ts;
    mrac->ym = (1.0f - am_ts) * mrac->ym + am_ts * setpoint;

	  // Buoc 2: Tinh Sai so thich nghi
    mrac->error_model = mrac->ym - feedback;

    // Sai so PI
    float error = setpoint - feedback;

	  // Buoc 3: Cap nhat tich phan Forward Euler
    mrac->integral += error * mrac->ts;
    mrac->integral = clamp(mrac->integral, mrac->integral_min, mrac->integral_max);

	  // Buoc 4: Tinh dau ra PI voi Kp, Ki hien tai
    mrac->output = (mrac->kp * error) + (mrac->ki * mrac->integral);
    mrac->output = clamp(mrac->output, mrac->output_min, mrac->output_max);

    // Xap xi do nhay: dphi_p = dy/dKp ~ e, dphi_i = dy/dKi ~ I
    mrac->dphi_p = error;          // Sensitivity cua u theo Kp
    mrac->dphi_i = mrac->integral; // Sensitivity cua u theo Ki

    // Buoc 5: Luat cap nhat MIT
    float em_ts = mrac->error_model * mrac->ts;
    mrac->kp -= mrac->gamma_p * em_ts * mrac->dphi_p;
    mrac->ki -= mrac->gamma_i * em_ts * mrac->dphi_i;

    // Gioi han Kp, Ki vao vung an toan
    mrac->kp = clamp(mrac->kp, mrac->kp_min, mrac->kp_max);
    mrac->ki = clamp(mrac->ki, mrac->ki_min, mrac->ki_max);

    // Buoc 6: Luu sai so
    mrac->prev_error = error;

    return mrac->output;
}

void mrac_reset(MRAC_t *mrac)
{
    mrac->integral = 0.0f;
    mrac->prev_error = 0.0f;
    mrac->ym = 0.0f;
    mrac->dphi_p = 0.0f;
    mrac->dphi_i = 0.0f;
    mrac->output = 0.0f;
    mrac->error_model = 0.0f;
}

void mrac_set_gamma(MRAC_t *mrac, float gamma_p, float gamma_i)
{
    mrac->gamma_p = gamma_p;
    mrac->gamma_i = gamma_i;
}
