#include <math.h>
#include "adc.h"
#include "delay.h"
#include "lis302dl.h"
#include "usart.h"

#define IRQ_IDLE 0
#define IRQ_DETECTED 1
#define IRQ_WAIT4LOW 2
#define IRQ_DEBOUNCE 3

volatile uint32_t g_irq_cnt = 0;
volatile uint8_t g_gpioa_irq_state = (IRQ_IDLE);
volatile uint32_t g_irq_timer = 0;

void getDataFromAngle(float, float);
void scaleWithPotenciometer();
uint16_t createCommand();
void serviceIRQA(void);
void initPushbutton();
void filterDataFromAccelerometer(int8_t*);

const int angleOffset = 9;
int8_t leftWheel;
int8_t rightWheel;
uint8_t directionMode = 1;
uint8_t stopMode = 0;

void init() {
  initSYSTIMER();
  initPushbutton();
  initAccelerometer();
  initADC1();
  initUSART2(USART2_BAUDRATE_9600);
  enIrqUSART2();
}

int main(void) {
  init();

  /* RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; */
  /* GPIOD->MODER |= 0x55000000; */
  /* GPIOD->OTYPER |= 0x00000000; */
  /* GPIOD->OSPEEDR |= 0xFF000000; */

  int8_t accelerometerData[3];
  float x, y, z, xAngle, yAngle;

  while (1) {
    serviceIRQA();

    filterDataFromAccelerometer(accelerometerData);

    if (checkCarPing) {
      /* getDataFromAccelerometer(accelerometerData); */

      x = accelerometerData[0];
      y = accelerometerData[1];
      z = accelerometerData[2];

      xAngle = atanf(x / sqrt(y * y + z * z)) * 180 / 3.14;
      yAngle = atanf(y / sqrt(x * x + z * z)) * 180 / 3.14;

      getDataFromAngle(xAngle, yAngle);

      scaleWithPotenciometer();

      sendCommand(createCommand());
      checkCarPing = 0;
    }
  }

  return 0;
}

uint16_t createCommand() {
  uint8_t header = 0x0C;
  if (stopMode) {
    header |= 0x02;
  }
  if (directionMode) {
    header |= 0x01;
  }

  /* uint8_t high = (leftWheel & 0x0f); */
  /* uint8_t low = (rightWheel & 0x0f); */

  /* uint8_t checksum = 0x0f - ((header + high + low) & 0x0f); */
  uint8_t checksum = 0x0f - ((header + leftWheel + rightWheel) & 0x0f);

  uint16_t command = 0x0;
  /* command |= (header << 12) | (high << 8) | (low << 4) | checksum; */
  command |= (header << 12) | (leftWheel << 8) | (rightWheel << 4) | checksum;

  return command;
}

void getDataFromAngle(float x, float y) {
  if (x > 0) {  // ispravan polozaj
    stopMode = 0;
    for (int i = 0; i < 10; ++i)
      if (y < -81 + i * angleOffset) {
        leftWheel = i;
        rightWheel = 9;
        return;
      }

    for (int i = 0; i < 10; ++i)
      if (y < 9 + i * angleOffset) {
        leftWheel = 9;
        rightWheel = 9 - i;
        return;
      }
  } else {  // neispravan polozaj
    if (y > 70) {
      stopMode = 0;
      leftWheel = 9;
      rightWheel = 0;
    } else if (y < -70) {
      stopMode = 0;
      leftWheel = 0;
      rightWheel = 9;
    } else {
      stopMode = 1;
      leftWheel = 0;
      rightWheel = 0;
    }
  }
}

void scaleWithPotenciometer() {
  leftWheel = (int8_t)round((float)leftWheel * getADC1() / 4095);
  rightWheel = (int8_t)round((float)rightWheel * getADC1() / 4095);
}

void initPushbutton() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER &= ~0x00000003;  //

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;  // enable clock on SYSCFG register
  SYSCFG->EXTICR[0] =
      SYSCFG_EXTICR1_EXTI0_PA;  // select PA 0 as interrupt source p259
  EXTI->IMR = EXTI_IMR_MR0;     // enable interrupt on EXTI_Line0
  EXTI->EMR &= ~EXTI_EMR_MR0;   // disable event on EXTI_Line0
  EXTI->RTSR = EXTI_RTSR_TR0;
  EXTI->FTSR = 0x00000000;

  NVIC_EnableIRQ(EXTI0_IRQn);
}

void EXTI0_IRQHandler(void) {
  if ((EXTI->PR & EXTI_PR_PR0) == EXTI_PR_PR0) {
    if (g_gpioa_irq_state == (IRQ_IDLE)) {
      /* directionMode = !directionMode; */

      if (directionMode)
        directionMode = 0;
      else
        directionMode = 1;
      g_gpioa_irq_state = (IRQ_DETECTED);
    }
    EXTI->PR = EXTI_PR_PR0;  // clear EXTI_Line0 interrupt flag
  }
}

void serviceIRQA(void) {
  switch (g_gpioa_irq_state) {
    case (IRQ_IDLE): {
      break;
    }
    case (IRQ_DETECTED): {
      g_irq_cnt++;
      g_gpioa_irq_state = (IRQ_WAIT4LOW);
      break;
    }
    case (IRQ_WAIT4LOW): {
      if ((GPIOA->IDR & 0x0001) == 0x0000) {
        g_gpioa_irq_state = (IRQ_DEBOUNCE);
        g_irq_timer = getSYSTIMER();
      }
      break;
    }
    case (IRQ_DEBOUNCE): {
      if (chk4TimeoutSYSTIMER(g_irq_timer, 50000) == (SYSTIMER_TIMEOUT)) {
        g_gpioa_irq_state = (IRQ_IDLE);
      }
    }
    default: { break; }
  }
}

void filterDataFromAccelerometer(int8_t* data) {
  uint16_t n = 0;
  int32_t xSum = 0, ySum = 0, zSum = 0;

  uint32_t gatherDataTime = getSYSTIMER();
  while (chk4TimeoutSYSTIMER(gatherDataTime, 200000) !=
         SYSTIMER_TIMEOUT) {  // 200ms
    getDataFromAccelerometer(data);
    xSum += data[0];
    ySum += data[1];
    zSum += data[2];
    ++n;
  }

  data[0] = xSum / n;
  data[1] = ySum / n;
  data[2] = zSum / n;
}
