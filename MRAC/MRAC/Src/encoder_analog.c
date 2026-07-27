/**
 * @file    encoder_analog.c
 * @brief   Thu vien doc encoder
 * @author  Bao Chien
 * @date    2026-05-13
 */

#include "encoder_analog.h"
#include "delay.h"

static uint16_t adc_convert_single(void)
{
    // Bat dau chuyen doi: ghi SWSTART = bit 22 cua CR2
    ADC1->CR2 |= (1 << 22);

    // Cho den khi EOC (End Of Conversion) = bit 1 cua SR duoc set
    while (!(ADC1->SR & (1 << 1)));

    // Xoa co EOC
    ADC1->SR &= ~(1 << 1);

    // Doc ket qua 12-bit tu Data Register
    return (uint16_t)(ADC1->DR & 0x0FFF);
}

void encoder_analog_init(void)
{
    // Bat clock GPIOA
    RCC->APB2ENR |= (1 << 2);

    // Cau hinh PA0 la Analog Input
    GPIOA->CRL &= ~(0xF << 0);

    // Bat clock ADC1
    RCC->APB2ENR |= (1 << 9);

    // Dat prescaler ADC: ADCPRE trong RCC->CFGR
    RCC->CFGR &= ~(0x3 << 14); // Xoa ADCPRE
    RCC->CFGR |= (0x2 << 14);  // ADCPRE = 10 => /6
		
		// Reset thanh ghi CR1 (khong dung che do scan/interrupt)
    ADC1->CR1 = 0;
		
		// Cau hinh trigger chuyen doi bang phan mem (SWSTART)
		// EXTTRIG  : cho phep trigger ngoai/phan mem
		// EXTSEL = 111: chon SWSTART lam nguon trigger
    ADC1->CR2 = (1 << 20) | (0x7 << 17);

		// Thoi gian lay mau cho kenh 0 (SMP0), chon 111 = thoi gian lay mau lon nhat
    ADC1->SMPR2 = (0x7 << 0);
		
		// So kenh trong chuoi chuyen doi: L = 0 => 1 conversion
    ADC1->SQR1 = 0;
		// Chon kenh can doc dua vao SQ1 (vi tri chuyen doi dau tien)
    ADC1->SQR3 = (ENCODER_ADC_CHANNEL & 0x1F);
		// Bat ADC lan 1 (ADON = 1), can bat 2 lan lien tiep
    ADC1->CR2 |= (1 << 0);

    for (volatile uint32_t i = 0; i < 100; i++)
        __NOP();

		// ADON = 1 lan thu hai -> bat dau khoi dong ADC
    ADC1->CR2 |= (1 << 0);
		
		// Reset calibration truoc khi hieu chuan
    ADC1->CR2 |= (1 << 3);
    while (ADC1->CR2 & (1 << 3)); // Doi RSTCAL tu dong ve 0

		// Bat dau qua trinh hieu chuan
    ADC1->CR2 |= (1 << 2);
		
		// Doi CAL tu dong ve 0, ket thuc hieu chuan
    while (ADC1->CR2 & (1 << 2)); 
}

uint16_t encoder_read_raw_avg(void)
{
    uint32_t sum = 0;

		// Cong don ENCODER_OVERSAMPLE lan doc ADC
    for (uint8_t i = 0; i < ENCODER_OVERSAMPLE; i++) {
        sum += adc_convert_single();
    }
		
		// Lay trung binh de ra gia tri raw on dinh hon
    return (uint16_t)(sum / ENCODER_OVERSAMPLE);
}

float encoder_read_relative_angle(uint16_t zero_raw)
{		
		// Do lech giua gia tri hien tai va gia tri zero
    uint16_t raw = encoder_read_raw_avg();
    int32_t delta = (int32_t)raw - (int32_t)zero_raw;
	
		// Quy doi tu don vi ADC (0..ENCODER_ADC_MAX) sang don vi goc (do)
    return ((float)delta / ENCODER_ADC_MAX) * ENCODER_ANGLE_MAX;
}

uint16_t encoder_calibrate_zero(void)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 32; i++) {
        sum += adc_convert_single();
    }
    return (uint16_t)(sum / 32);
}
