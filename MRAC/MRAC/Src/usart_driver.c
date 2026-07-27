/*
 * usart_driver.c
 *
 * Thu vien giao tiep USART
 * Created on: Oct 16, 2025
 * Author: Dang Bao Chien
 *
 */

#include "usart_driver.h"
#include "gpio_driver.h"
 
void usart1_init(uint32_t baudrate)
{
	gpio_enable_clock(GPIOA);
	
	// Bat Clock cho USART1
	RCC->APB2ENR |= (1 << 14);
	
	// Bat Clock cho AFIO
	RCC->APB2ENR |= (1 << 0);
	
	// Cau hinh PA9 - TX (Output 10MHz - AF PushPull)
	gpio_init(GPIOA, 9, GPIO_MODE_OUTPUT_10M, GPIO_CNF_AF_PP);
	// Cau hinh PA10 - TX (Input Floating)
	gpio_init(GPIOA, 10, GPIO_MODE_INPUT, GPIO_CNF_FLOATING);
	
	
	// Tinh gia tri BRR (Baud Rate Register)
  	// BRR = fPCLK / (16 * baudrate)
  	// fPCLK = 72MHz cho USART1 (APB2)
	uint32_t usartdiv = 72000000 / baudrate;
	USART1->BRR = usartdiv;
	
	USART1->CR1 = 0; 		   // Reset 
	USART1->CR1 |= (1 << 13);  // UE = 1: Bat USART
	USART1->CR1 |= (1 << 3);   // TE = 1: Bat truyen (Transmitter)
	USART1->CR1 |= (1 << 2);   // RE = 1: Bat nhan (Receiver)
	
	// Word length = 8 bit (M = 0)
  	// Parity = None (PCE = 0)
  	// Stop bit = 1 (trong CR2)
	
	USART1->CR2 = 0;  // Stop bit = 1
	USART1->CR3 = 0;  // Khong dung flow control
	
	USART1->CR1 |= (1 << 5);  // RXNEIE = 1: Enable RXNE interrupt
	
	// **Enable USART1 interrupt trong NVIC**
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 2);  // Priority 2
}


void usart_send_char(char c)
{	
	// Doi TXE (Transmit Data Register Empty) = 1
	// Bit 7 cua SR: 1 = buffer trong, san sang gui
	while (!(USART1->SR & (1 << 7)));
	
	// Ghi ky tu vao Data Register
	USART1->DR = c;
}

void usart_send_string(const char* str)
{	
	// Gui tung ky tu cho den khi gap '\0'
	while (*str) {
		usart_send_char(*str++);
	}
}
void usart_send_number(int32_t num)
{
	char buffer[12];
	int i = 0;
	
	if (num < 0) {
		usart_send_char('-');
		num = -num;
	}
	
	if (num == 0) {
		usart_send_char('0');
		return;
	}
	
	// Chuyen so thanh chuoi
	while (num > 0) {
		buffer[i++] = (num % 10) + '0';
		num /= 10;
	}
	
	// Gui chuoi theo thu tu
	while (i > 0) {
		usart_send_char(buffer[--i]);
	}
}
	
char usart_receive_char(void)
{
	// Doi RXNE (Read Data Register Not Empty) = 1
  	// Bit 5 cua SR: 1 = da nhan duoc du lieu
  	while (!(USART1->SR & (1 << 5)));
    
  	// Doc ky tu tu Data Register
  	return (char)(USART1->DR & 0xFF);
}

void send_float_usart(float val)
{
    if (val < 0.0f) {
        usart_send_char('-');
        val = -val;
    }
    usart_send_number((int32_t)val);
    usart_send_char('.');
    int32_t frac = (int32_t)((val - (float)(int32_t)val) * 100.0f);
    if (frac < 10)
        usart_send_char('0');
    usart_send_number(frac);
}
