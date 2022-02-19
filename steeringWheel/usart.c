#include "usart.h"

volatile uint8_t checkCarPing = 0; 

void initUSART2(uint32_t baudrate) {
  // USART2: PA2 -> TX & PA3 -> RX

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  GPIOA->MODER |= (GPIO_MODER_MODER2_1) | (GPIO_MODER_MODER3_1);
  GPIOA->AFR[0] |= 0x00007700;

  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2_1;
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3_1;

  USART2->BRR = baudrate;
  USART2->CR1 = USART_CR1_UE | USART_CR1_TE;
}

void enIrqUSART2(void) {
  USART2->CR1 &= ~(USART_CR1_UE);

  NVIC_EnableIRQ(USART2_IRQn);
  USART2->CR1 |= (USART_CR1_UE) | (USART_CR1_RE) | (USART_CR1_RXNEIE);
}

void disIrqUSART2(void) {
  USART2->CR1 &= ~((USART_CR1_UE) | (USART_CR1_RXNEIE));

  NVIC_DisableIRQ(USART2_IRQn);
  USART2->CR1 |= (USART_CR1_UE);
}

void putcharUSART2(uint8_t data) {  /// print one character to USART2
  while (!(USART2->SR & USART_SR_TC))
    ;

  USART2->DR = data;
}

void sendCommand(uint16_t command) {
  putcharUSART2(command >> 8);		//high
  putcharUSART2(command & 0xff);	//low
}

void USART2_IRQHandler(void) {
  if (USART2->SR & (USART_SR_RXNE)) {
   uint8_t command = USART2->DR;
    if(command == 0xA5){
     checkCarPing = 1;
   } 
  }
}

