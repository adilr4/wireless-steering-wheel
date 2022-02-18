#include <math.h>
#include "delay.h"
#include "lis302dl.h"
#include "usart.h"

void getDataFromAngle(float, float);
void scaleWithPotenciometer();
uint16_t createCommand();

const int angleOffset = 9;
int8_t leftWheel;
int8_t rightWheel;
uint8_t directionMode = 1;
uint8_t stopMode = 0;

void init() {
  initAccelerometer();
  initUSART2(USART2_BAUDRATE_9600);
  enIrqUSART2();
}

int main(void) {
  init();

  int8_t accelerometerData[3];

  while (1) {
    getDataFromAccelerometer(accelerometerData);

    float x = accelerometerData[0];
    float y = accelerometerData[1];
    float z = accelerometerData[2];

    float xAngle = atanf(x / sqrt(y * y + z * z)) * 180 / 3.14;
    float yAngle = atanf(y / sqrt(x * x + z * z)) * 180 / 3.14;

    getDataFromAngle(xAngle, yAngle);

    // scaleWithPotenciometer();

    sendCommand(createCommand());

    delay_ms(1000);
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

  uint8_t high = (leftWheel & 0x0f);
  uint8_t low = (rightWheel & 0x0f);

  uint8_t checksum = 0x0f - ((header + high + low) & 0x0f);

  uint16_t command = 0x0;
  command |= (header << 12) | (high << 8) | (low << 4) | checksum;

  return command;
}

void getDataFromAngle(float x, float y) {
  if (x > 0) {  // ispravan polozaj
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
      leftWheel = 9;
      rightWheel = 0;
    } else if (y < -70) {
      leftWheel = 0;
      rightWheel = 9;
    } else {
      // stani
    }
  }
}

void scaleWithPotenciometer() {}

