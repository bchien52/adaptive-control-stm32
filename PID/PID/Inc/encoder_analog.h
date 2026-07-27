/**
 * @file    encoder_analog.h
 * @brief   Thu vien doc encoder analog qua ADC
 * @author  Bao Chien
 * @date    2026-05-13
 */

#ifndef ENCODER_ANALOG_H
#define ENCODER_ANALOG_H

#include "stdint.h"
#include "stm32f10x.h"

#define ENCODER_ADC_CHANNEL 0

// Do phan giai ADC: 12-bit => 4096 buoc
#define ENCODER_ADC_MAX 4095.0f

#define ENCODER_ANGLE_MAX 330.0f

#define ENCODER_OVERSAMPLE 8

/**
 * @brief  Khoi tao ADC1 doc encoder analog
 * @note   Cau hinh ADC1 single conversion, PA0 la input analog
 *         Bat clock GPIOA, ADC1, calibrate ADC
 */
void encoder_analog_init(void);

/**
 * @brief  Doc gia tri ADC thu (0..4095)
 * @retval Gia tri ADC 12-bit [0..4095]
 */
uint16_t encoder_read_raw(void);

/**
 * @brief  Doc gia tri ADC co lay trung binh (giam nhieu)
 * @retval Gia tri ADC 12-bit trung binh [0..4095]
 */
uint16_t encoder_read_raw_avg(void);

/**
 * @brief  Doc goc hien tai cua encoder (do)
 * @retval Goc tinh bang do [0.0 .. ENCODER_ANGLE_MAX]
 */
float encoder_read_angle(void);

/**
 * @brief  Doc goc va tru goc zero ban dau (goc tuong doi)
 * @param  zero_raw  Gia tri ADC raw tai vi tri goc 0 do (lay luc khoi tao)
 * @retval Goc tuong doi co the am hoac duong (do)
 */
float encoder_read_relative_angle(uint16_t zero_raw);

/**
 * @brief  Doc goc zero ban dau luc khoi dong
 * @retval Gia tri ADC raw tai vi tri hien tai (luu lam zero)
 */
uint16_t encoder_calibrate_zero(void);

#endif /* ENCODER_ANALOG_H */
