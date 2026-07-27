#include "delay.h"
#include "encoder_analog.h"
#include "gpio_driver.h"
#include "mrac_controller.h"
#include "stm32f10x.h"
#include "usart_driver.h"

// Cau hinh PWM
#define PWM_PSC 71U
#define PWM_ARR 999U

// Cau hinh TIM2, Ts = 5ms
#define TIM2_PSC 71U
#define TIM2_ARR 4999U

MRAC_t mrac;
uint16_t zero_raw = 0;
float setpoint = 8.0f; // Goc dat
float angle = 0.0f;    // Goc thuc te
volatile uint8_t timer_flag = 0;

/** Ham khoi tao PWM */ 
void pwm_init(void)
{
    // Bat TIM1
    RCC->APB2ENR |= (1 << 11);

    gpio_init(GPIOA, 8, GPIO_MODE_OUTPUT_50M, GPIO_CNF_AF_PP);

    TIM1->PSC = PWM_PSC;
    TIM1->ARR = PWM_ARR;
    TIM1->CNT = 0;
    TIM1->CCR1 = 0;

    // OC1M = 110 (PWM mode 1), OC1PE = 1 (preload enable)
    TIM1->CCMR1 &= ~(0xFF);
    TIM1->CCMR1 |= (0x68);

    // CC1E: enable capture/compare 1 output
    TIM1->CCER |= (1 << 0);

    // MOE: Main Output Enable
    TIM1->BDTR |= (1 << 15);

    // CEN: bat TIM1
    TIM1->CR1 |= (1 << 0);

    // UG: tao update event de nap PSC, ARR, CCR
    TIM1->EGR |= (1 << 0);
}

/** Ham set duty pwm */
void pwm_set(float duty)
{
    if (duty > 1.0f) duty = 1.0f;
    if (duty < 0.0f) duty = 0.0f;
    TIM1->CCR1 = (uint16_t)(duty * (float)(PWM_ARR + 1U) + 0.5f);
}

/** Ham khoi tao TIM2 */
void timer2_init(void)
{
    // Bat clock TIM2
    RCC->APB1ENR |= (1 << 0);

    TIM2->PSC = TIM2_PSC;
    TIM2->ARR = TIM2_ARR;
    TIM2->CNT = 0;
    TIM2->SR = 0;

    // UIE: Update Interrupt Enable
    TIM2->DIER |= (1 << 0);

    // CEN: Bat TIM2
    TIM2->CR1 |= (1 << 0);

    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);
}

/** Ham ngat TIM2 - 5ms */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & (1 << 0)) {
        TIM2->SR &= ~(1 << 0);  // Xoa co tran
        timer_flag = 1;
    }
}

/** Ham gui du lieu qua UART toi Serial Osciloscope */
void send_data(void)
{
    send_float_usart(setpoint);
    usart_send_char(',');
    send_float_usart(angle);
    usart_send_char(',');

    // Gui y_m (nhan voi ENCODER_ANGLE_MAX de chuyen ve do)
    send_float_usart(mrac.ym * ENCODER_ANGLE_MAX);
    usart_send_char(',');
    send_float_usart(mrac.kp);
    usart_send_char(',');
    send_float_usart(mrac.ki);
    usart_send_string("\r\n");
}

/** Chuong trinh chinh */
int main(void)
{
    SystemInit();

		// Khoi tao Clock GPIO
    gpio_enable_clock(GPIOA);
    gpio_enable_clock(GPIOB);
    gpio_enable_clock(GPIOC);

    gpio_init(GPIOC, 13, GPIO_MODE_OUTPUT_2M, GPIO_CNF_GP_PP);
    gpio_write(GPIOC, 13, 1);

		// Khoi tao UART
    usart1_init(115200);
		
		// Khoi tao Encoder
    encoder_analog_init();

    delay_ms(100);
    zero_raw = encoder_calibrate_zero();
		
		// Khoi tao PWM
    pwm_init();
    pwm_set(0.0f);
		
		// Khoi tao TIM2
		timer2_init();

		// Khoi tao MRAC
    mrac_init(&mrac, MRAC_KP_INIT, MRAC_KI_INIT, MRAC_GAMMA_P, MRAC_GAMMA_I, MRAC_AM, MRAC_TS);

		// Gui qua Serial Osciloscope
    usart_send_string("SP,Angle,Ym,Kp,Ki\r\n");

    while (1) {
        if (timer_flag) {
            timer_flag = 0;

            angle = encoder_read_relative_angle(zero_raw);

            float sp_norm = setpoint / ENCODER_ANGLE_MAX;
            float ang_norm = angle / ENCODER_ANGLE_MAX;

            // Tinh toan MRAC-MIT
            float duty = mrac_compute(&mrac, sp_norm, ang_norm);

            // Dat PWM
            pwm_set(duty);

            // Gui du lieu len PC moi 5ms
            send_data();
        }
    }
}
