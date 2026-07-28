/**
 * @file    mrac_controller.h
 * @brief   Bo dieu khien thich nghi MRAC - Luat MIT Rule
 * @author  Dang Bao Chien
 * @date    2026-05-13
 * 
 * @details
 *		Thuat toan MRAC-MIT (Model Reference Adaptive Control - MIT Rule):
 *
 *   [1] MO HINH MAU (Reference Model):
 *       y_m(k) = (1 - am*Ts)*y_m(k-1) + am*Ts*r(k)
 *       Chon am = 1/tau_m, tau_m = 0.3s
 *
 *   [2] DIEU KHIEN PI (co tham so thich nghi):
 *       e(k)  = r(k) - y(k)
 *       I(k)  = I(k-1) + e(k)*Ts
 *       u(k)  = Kp(k)*e(k) + Ki(k)*I(k)
 *
 *   [3] LUAT MIT (gradient descent tren J = 0.5*e_m^2):
 *       e_m(k)  = y_m(k) - y(k)
 *       dJ/dKp  = e_m * (dy/dKp) ~ e_m * e(k)
 *       dJ/dKi  = e_m * (dy/dKi) ~ e_m * I(k)
 *
 *       Kp(k+1) = Kp(k) - gamma_p * Ts * e_m * e(k)
 *       Ki(k+1) = Ki(k) - gamma_i * Ts * e_m * I(k)
 *
 *   [4] CLAMP :
 *       Kp in [KP_MIN .. KP_MAX]
 *       Ki in [KI_MIN .. KI_MAX]
 *       I  in [INTEGRAL_MIN .. INTEGRAL_MAX]
 *       u  in [OUTPUT_MIN   .. OUTPUT_MAX  ]
 *
 */

#ifndef MRAC_CONTROLLER_H
#define MRAC_CONTROLLER_H

#include "stdint.h"

#define MRAC_AM 3.33f        /**< Cuc cua mo hinh mau (rad/s) */
#define MRAC_TS 0.005f       /**< Chu ky lay mau Ts = 5ms */
#define MRAC_GAMMA_P 100     /**< Toc do hoc cho Kp */
#define MRAC_GAMMA_I 50      /**< Toc do hoc cho Ki */
#define MRAC_KP_MIN 0.1f     /**< Kp toi thieu */
#define MRAC_KP_MAX 5.0f     /**< Kp toi da */
#define MRAC_KI_MIN 0.5f     /**< Ki toi thieu */
#define MRAC_KI_MAX 30.0f    /**< Ki toi da */
#define MRAC_KP_INIT 0.702f  /**< Kp khoi tao */
#define MRAC_KI_INIT 6.356f  /**< Ki khoi tao */
#define MRAC_OUTPUT_MAX 1.0f
#define MRAC_OUTPUT_MIN 0.0f
#define MRAC_INTEGRAL_MAX 0.2f
#define MRAC_INTEGRAL_MIN -0.2f

typedef struct {
    float kp;                /**< Kp hien tai (thich nghi) */
    float ki;                /**< Ki hien tai (thich nghi) */
    float ts;                /**< Chu ky lay mau (s) */
    float gamma_p;           /**< Toc do hoc Kp */
    float gamma_i;           /**< Toc do hoc Ki */
    float am;                /**< Cuc mo hinh mau */
    float integral;          /**< Tich phan hien tai */
    float prev_error;        /**< Sai so vong truoc */
    float ym;                /**< Dap ung mo hinh mau y_m(k) */
    float dphi_p;            /**< Gradient cua e theo Kp [= e(k)] */
    float dphi_i;            /**< Gradient cua e theo Ki [= I(k)] */
    float output;            /**< Dau ra PI */
    float error_model;       /**< Sai so thich nghi e_m = y_m - y */
	
    float kp_min, kp_max;			   /**< Cac bien luu tru gioi han cua Kp */
    float ki_min, ki_max;			   /**< Cac bien luu tru gioi han cua Ki */
    float integral_min, integral_max;  /**< Cac bien luu tru gioi han cua khau tich phan */
    float output_min, output_max;	   /**< Cac bien luu tru gioi han ngo ra */
} MRAC_t;

/**
 * @brief Khoi tao bo dieu khien MRAC voi cac tham so ban dau.
 * 
 * @param mrac Con tro tro den bien cau truc MRAC_t.
 * @param kp_init He so Kp ban dau.
 * @param ki_init He so Ki ban dau.
 * @param gamma_p Toc do hoc cap nhat Kp.
 * @param gamma_i Toc do hoc cap nhat Ki.
 * @param am Thong so mo hinh mau.
 * @param ts Thoi gian lay mau (s).
 */
void mrac_init(MRAC_t *mrac, float kp_init, float ki_init, float gamma_p, float gamma_i, float am, float ts);


/**
 * @brief Tinh toan tin hieu dieu khien ngo ra dong thoi cap nhat thich nghi Kp, Ki.
 * @param mrac Con tro tro den bien cau truc MRAC_t.
 * @param setpoint Gia tri dat mong muon (r).
 * @param feedback Gia tri thuc te đo duoc (y).
 * @return float Gia tri dieu khien ngo ra u(k) da duoc gioi han.
 */
float mrac_compute(MRAC_t *mrac, float setpoint, float feedback);

/**
 * @brief Reset lai cac gia tri trang thai cua he thong.
 * @param mrac Con tro tro den bien cau truc MRAC_t.
 * @note Kp va Ki hien tai duoc giu nguyen.
 */
void mrac_reset(MRAC_t *mrac);


/**
 * @brief Thay doi toc do hoc (gamma) luc chuong trinh dang chay.
 * @param mrac Con tro tro den bien cau truc MRAC_t.
 * @param gamma_p Toc do hoc moi cua Kp.
 * @param gamma_i Toc do hoc moi cua Ki.
 */
void mrac_set_gamma(MRAC_t *mrac, float gamma_p, float gamma_i);

#endif /* MRAC_CONTROLLER_H */
