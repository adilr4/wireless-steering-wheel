#include "adc.h"

void initADC1(void) {  /// i
  // wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
  // ADC PA1 -> ADC123_IN1
  //------------------------------------------------------------------

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  GPIOA->MODER |= GPIO_MODER_MODER1;
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1;

  ADC1->CR1 = 0x00000800;    // 12-bit resolution (15 ADCCLK cycles), 1 channel
                             // discontinious mode on regular channels
                             // single conversion mode
  ADC1->SMPR1 = 0x00000000;  // conversion time 3 cycles of ADCCLK clock!
  ADC1->SMPR2 = 0x000000ff;  // conversion time 3 cycles of ADCCLK clock!
  ADC1->SQR1 = 0x000000000;  // select one channel in regular sequence
  ADC1->SQR2 = 0x000000000;
  ADC1->SQR3 = 0x000000001;  // select channel 1
  ADC1->CR2 = 0x000000400;
  ADC->CCR = 0x00010000;  // ADC clock 84/4 = 21MHz!

  ADC1->CR2 |= ADC_CR2_ADON;  // turn on the ADC
}

uint16_t getADC1(void) {
  uint16_t adc_data;

  adc_data = ADC1->DR;  // clear the EOC flag

  ADC1->CR2 |= ADC_CR2_SWSTART;  // start conversion of regular channels
  while ((ADC1->SR & ADC_SR_EOC) != ADC_SR_EOC)
    ;                   // wait for regular channel to complete the conversion
  adc_data = ADC1->DR;  // get the data

  return adc_data;
}
