/**
 * @file    usart_driver.h
 * @brief   Thu vien giao tiep USART
 * @author  Dang Bao Chien
 * @date    2025-10-16
 */

#ifndef USART_DRIVER_H
#define USART_DRIVER_H

#include "stm32f10x.h"
#include <stdint.h>
#include <stdio.h>

/** USART Baudrate  */
#define USART_BAUDRATE_9600 9600     /**< Toc do Baud 9600 */
#define USART_BAUDRATE_115200 115200 /**< Toc do Baud 115200 */

/**
 * @brief  Khoi tao USART1 va cau hinh GPIO.
 *
 * Cau hinh chi tiet:
 * - Clock: Cap clock cho USART1 (APB2 72MHz) va GPIOA.
 * - GPIO:
 * + PA9 (TX): Alternate Function Push-Pull (10MHz).
 * + PA10 (RX): Input Floating.
 * - Baudrate: Tu dong tinh toan he so chia (BRR) dua tren fPCLK.
 * - Khung truyen: 8 bit data, 1 stop bit, No parity.
 * - Ngat: Bat ngat nhan (RXNE) va thiet lap muc uu tien (Priority 2).
 *
 * @param  baudrate: Toc do baud mong muon (VD: 9600, 115200).
 * @retval None
 */
void usart1_init(uint32_t baudrate);

/**
 * @brief  Truyen mot ky tu qua USART1.
 *
 * Ham hoat dong o che do Blocking:
 * 1. Cho den khi thanh ghi truyen (TXE) trong.
 * 2. Ghi ky tu vao thanh ghi du lieu (DR).
 *
 * @param  c: Ky tu can truyen.
 * @retval None
 */
void usart_send_char(char c);

/**
 * @brief  Truyen mot chuoi ky tu qua USART1.
 *
 * Gui lan luot tung ky tu trong chuoi cho den khi gap ky tu ket thuc '\0'.
 *
 * @param  str: Con tro toi chuoi ky tu (null-terminated).
 * @retval None
 */
void usart_send_string(const char *str);

/**
 * @brief  Truyen mot so nguyen (co dau) qua USART1.
 *
 * Xu ly:
 * - Neu la so am: Gui dau '-' va chuyen sang so duong.
 * - Chuyen doi gia tri so thanh chuoi ASCII.
 * - Khong tu dong them ky tu xuong dong.
 *
 * @param  num: So nguyen 32-bit can truyen.
 * @retval None
 */
void usart_send_number(int32_t num);


/**
 * @brief  Nhan mot ky tu tu USART1 (Blocking).
 *
 * Ham cho den khi co du lieu trong buffer nhan (RXNE = 1).
 *
 * @return Ky tu nhan duoc tu thanh ghi DR.
 */
char usart_receive_char(void);

/**
 * @brief  Truyen mot so thuc (float) qua USART1, dinh dang X.XX.
 *
 * @param  val: Gia tri float can truyen.
 * @retval None
 */
void send_float_usart(float val);

#endif // USART_DRIVER_H
