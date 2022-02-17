#include "stm32f4xx.h"
#include "usart.h"

uint16_t d1[10] = {0x53E8, 0x53D4, 0x53C0, 0x53AC, 0x5398,
                   0x5384, 0x5370, 0x535C, 0x5348, 0x5334};  // 21480-21300
uint16_t d2[10] = {0x5410, 0x5424, 0x5438, 0x544C, 0x5460,
                   0x5474, 0x5488, 0x549C, 0x54B0, 0x54C4};  // 21520-21700

void initServoMotors();

void init() {
  initServoMotors();
  initUSART2(USART2_BAUDRATE_9600);
  enIrqUSART2();
}

int main(void) {
  init();

  while (1) {
    if (g_usart2_ridx != g_usart2_widx) {
      USART2->DR = g_usart2_buffer[g_usart2_ridx++];
      if (g_usart2_ridx >= (USART2_BUFFER_SIZE)) {
        g_usart2_ridx = 0;
      }
    }
  }
}

void initServoMotors() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  //
  GPIOB->MODER |= 0x00002200;           // PB4 PB6
  GPIOB->OTYPER |= 0x00000000;          //
  GPIOB->AFR[0] |= 0x02020000;          //

  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // enable TIM3 on APB1
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;  // enable TIM4 on APB1

  // 84MHz/84 = 1MHz -> count each 1us
  TIM3->PSC = 0x0054 - 0x0001;  // set TIM3 counting prescaler
  TIM4->PSC = 0x0054 - 0x0001;  // set TIM4 counting prescaler

  TIM3->ARR = 0x53FC;  // period of the PWM 21.7ms
  TIM4->ARR = 0x53FC;  // period of the PWM 21.3ms

  TIM3->CCR1 = 20010;  // period of 20ms
  TIM4->CCR1 = 20010;  // period of 20ms

  TIM3->CCMR1 |= (TIM_CCMR1_OC1PE) | (TIM_CCMR1_OC1M_2) | (TIM_CCMR1_OC1M_1) |
                 (TIM_CCMR1_OC1M_0);
  TIM4->CCMR1 |= (TIM_CCMR1_OC1PE) | (TIM_CCMR1_OC1M_2) | (TIM_CCMR1_OC1M_1) |
                 (TIM_CCMR1_OC1M_0);

  // set active mode high for pulse polarity
  TIM3->CCER &= ~(TIM_CCER_CC1P);
  TIM3->CR1 |= (TIM_CR1_ARPE) | (TIM_CR1_URS);

  TIM4->CCER &= ~(TIM_CCER_CC1P);
  TIM4->CR1 |= (TIM_CR1_ARPE) | (TIM_CR1_URS);

  // update event, reload all config
  TIM3->EGR |= TIM_EGR_UG;
  TIM4->EGR |= TIM_EGR_UG;
  // activate capture compare mode
  TIM3->CCER |= (TIM_CCER_CC1E);
  TIM4->CCER |= (TIM_CCER_CC1E);
  // start counter
  TIM3->CR1 |= TIM_CR1_CEN;
  TIM4->CR1 |= TIM_CR1_CEN;
}
