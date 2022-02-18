#include "delay.h"

void delay_ms(uint32_t ms) {  /// delay in ms

  RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;  //
  TIM12->PSC = 0x0054 - 0x0001;         // APB1@42MHz
                                        //
  TIM12->ARR = 0x03E8;                  // reload 1000 us
  TIM12->CR1 = 0x0084;                  // ARPE On, CMS disable, Up counting

  TIM12->EGR |= TIM_EGR_UG;   // reload all config p363
  TIM12->CR1 |= TIM_CR1_CEN;  // start counter
  while (ms > 0) {
    while ((TIM12->SR & TIM_SR_UIF) == 0x0000)
      ;  // wait for update event

    TIM12->SR &= ~TIM_SR_UIF;  // clear the update event interrupt flag
    ms--;
  }
  TIM12->CR1 &= ~TIM_CR1_CEN;            // stop counter
  RCC->APB1ENR &= ~RCC_APB1ENR_TIM12EN;  // disable TIM
}

void delay_us(uint32_t us) {  /// delay in us

  RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;  //
  TIM12->PSC = 0x0001 - 0x0001;         //
                                        //
  TIM12->ARR = 0x0054;                  // reload value set to 1 us
  TIM12->CR1 = 0x0084;                  // ARPE On, CMS disable, Up counting
                                        // UEV disable, TIM4 enable(p392)

  TIM12->EGR |= TIM_EGR_UG;   // reload all config p363
  TIM12->CR1 |= TIM_CR1_CEN;  // start counter
  while (us > 0) {
    while ((TIM12->SR & TIM_SR_UIF) == 0x0000)
      ;  // wait for update event

    TIM12->SR &= ~TIM_SR_UIF;  // clear the update event interrupt flag
    us--;
  }
  TIM12->CR1 &= ~TIM_CR1_CEN;            // stop counter
  RCC->APB1ENR &= ~RCC_APB1ENR_TIM12EN;  // disable TIM4
}

void initSYSTIMER(void) {
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  //
  TIM2->PSC = 0x0054 - 0x0001;         //
                                       //
  TIM2->ARR = 0xFFFFFFFF;              //
  TIM2->CR1 = 0x0084;                  //
                                       //
  TIM2->CR2 = 0x0000;
  TIM2->CNT = 0x00000000;    //
  TIM2->EGR |= TIM_EGR_UG;   //
  TIM2->CR1 |= TIM_CR1_CEN;  //
}

uint32_t getSYSTIMER(void) {
  uint32_t time = TIM2->CNT;
  return time;
}

uint8_t chk4TimeoutSYSTIMER(uint32_t btime, uint32_t period) {
  uint32_t time = TIM2->CNT;
  if (time >= btime) {
    if (time >= (btime + period))
      return (SYSTIMER_TIMEOUT);
    else
      return (SYSTIMER_KEEP_ALIVE);
  } else {
    uint32_t utmp32 = 0xFFFFFFFF - btime;
    if ((time + utmp32) >= period)
      return (SYSTIMER_TIMEOUT);
    else
      return (SYSTIMER_KEEP_ALIVE);
  }
}
