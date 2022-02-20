#include "delay.h"
#include "stm32f4xx.h"
#include "usart.h"

uint16_t d1[16], d2[16];
uint32_t pausePeriod, stopPeriod;

void initServoMotors();
void parseMessage();
uint8_t validateChecksum(uint8_t, uint8_t);
void initPWMvalues();

void init() {
  initPWMvalues();
  initServoMotors();
  initUSART2(USART2_BAUDRATE_9600);
  enIrqUSART2();
  initSYSTIMER();
}

int main(void) {
  init();

  uint32_t connectionLostTime;
  uint8_t connectionLost = 0;

  // for debug
  /* RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; */
  /* GPIOD->MODER |= 0x55000000; */
  /* GPIOD->OTYPER |= 0x00000000; */
  /* GPIOD->OSPEEDR |= 0xFF000000; */

  while (1) {
    pingSteeringWheel();
    if (g_usart2_ridx != g_usart2_widx &&
        (g_usart2_ridx + 1) != g_usart2_widx) {
      parseMessage();
      connectionLost = 0;
    } else {
      if (connectionLost) {
        if (chk4TimeoutSYSTIMER(connectionLostTime, 3000000) ==
            SYSTIMER_TIMEOUT) {
          TIM3->ARR = stopPeriod;
          TIM4->ARR = stopPeriod;
          connectionLost = 0;
        }
      } else {
        connectionLostTime = getSYSTIMER();
        connectionLost = 1;
      }
    }
  }
}

void parseMessage() {
  uint8_t low, high;
  high = g_commandsBuffer[g_usart2_ridx++];
  low = g_commandsBuffer[g_usart2_ridx++];

  if (validateChecksum(low, high)) {
    if (high & 0x20) {
      TIM3->ARR = stopPeriod;
      TIM4->ARR = stopPeriod;
    } else {
      if (high & 0x10) {
        TIM3->ARR = d1[high & 0x0f];
        TIM4->ARR = d2[low >> 4];
      } else {
        TIM3->ARR = d2[high & 0x0f];
        TIM4->ARR = d1[low >> 4];
      }
    }
  }
  if (g_usart2_ridx >= (USART2_BUFFER_SIZE)) {
    g_usart2_ridx = 0;
  }
}

uint8_t validateChecksum(uint8_t low, uint8_t high) {
  uint8_t r = (((low >> 4) + (low & 0x0f) + (high >> 4) + (high & 0x0f)) &
               0x0f) == 0x0f;

  return r;
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

  TIM3->ARR = stopPeriod;  // period of the PWM 21.7ms
  TIM4->ARR = stopPeriod;  // period of the PWM 21.3ms

  TIM3->CCR1 = pausePeriod;  // period of 20ms
  TIM4->CCR1 = pausePeriod;  // period of 20ms

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

void initPWMvalues() {
  pausePeriod = 20005;
  stopPeriod = pausePeriod + 1500;

  for (int i = 0; i < 16; ++i) {
    d1[i] = stopPeriod - i * 200 / 16;
    d2[i] = stopPeriod + i * 200 / 16;
  }
}
