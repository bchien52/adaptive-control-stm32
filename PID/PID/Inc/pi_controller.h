/**
 * @file    pi_controller.h
 * @brief   Thu vien PI Controller
 * @author  Dang Bao Chien
 * @date    2026-05-13
 *
 * @details
 * 		Thuat toan PI:
 * 			e(k) = setpoint - feedback
 * 			I(k) = I(k-1) + e(k) * Ts
 * 			u(k) = Kp*e(k) + Ki*I(k)
 *
 * @note 
 *		Plant: G(s) = 25.13 / (s^2 + 4.999s + 19.96)
 * 		Thong so mac dinh tu MATLAB/Simulink PID Tuner:
 * 			Kp = 1.005 | Ki = 8.0 | Ts = 5ms
 *
 */

#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include "stdint.h"

#define PID_KP 0.702f   /**< Kp tu MATLAB = 0.702 */
#define PID_KI 6.356f   /**< Ki tu MATLAB = 6.356 */
#define PID_TS 0.005f   /**< Ts = 5ms */

#define PID_INTEGRAL_MAX 0.2f   /**< I_max = 1.0/Ki ~ 0.2 */
#define PID_INTEGRAL_MIN -0.2f  /**< Doi xung */
#define PID_OUTPUT_MAX 1.0f     /**< Duty toi da 100% */
#define PID_OUTPUT_MIN 0.0f     /**< Duty toi thieu 0% */

typedef struct {
    float kp;	           /**< He so Kp */
    float ki;	           /**< He so Ki */
    float ts;            /**< Thoi gian lay mau */
    float integral;      /**< Gia tri tich luy cua khau tich phan */
    float prev_error;    /**< Sai so o buoc truoc do */
    float output;				 /**< Gia tri ngo ra sau khi tinh toan */
    float integral_max;  /**< Gioi han tren cua khau tich phan */
    float integral_min;  /**< Gioi han duoi cua khau tich phan */
    float output_max;    /**< Gioi han tren cua ngo ra */
    float output_min;    /**< Gioi han duoi cua ngo ra */
} PID_t;

/**
 * @brief Khoi tao thong so cho bo dieu khien PI.
 * @param pid Con tro tro den bien cau truc PID.
 * @param kp He so Kp.
 * @param ki He so Ki.
 * @param ts Thoi gian lay mau (Ts).
*/
void  pid_init(PID_t *pid, float kp, float ki, float ts);

/**
 * @brief Tinh toan tin hieu ngo ra cua bo PI (thuc thi theo moi chu ky lay mau).
 * @param pid Con tro tro den bien cau truc PID.
 * @param setpoint Gia tri mong muon.
 * @param feedback Gia tri thuc te do duoc tu cam bien.
 * @return float Gia tri ngo ra dieu khien (duty cycle) da duoc gioi han.
 */
float pid_compute(PID_t *pid, float setpoint, float feedback);

/**
 * @brief Xoa cac trang thai cu cua bo PI.
 * @param pid Con tro tro den bien cau truc PID.
 */
void  pid_reset(PID_t *pid);

/**
 * @brief Cai dat gioi han cho khau tich phan.
 * @param pid Con tro tro den bien cau truc PID.
 * @param max Gioi han tren cua khau tich phan.
 * @param min Gioi han duoi cua khau tich phan.
 */
void  pid_set_integral_limit(PID_t *pid, float max, float min);

/**
 * @brief Cai dat gioi han tin hieu ngo ra.
 * @param pid Con tro tro den bien cau truc PID.
 * @param max Gioi han tren cua ngo ra.
 * @param min Gioi han duoi cua ngo ra.
 */
void  pid_set_output_limit(PID_t *pid, float max, float min);

#endif /* PI_CONTROLLER_H */
