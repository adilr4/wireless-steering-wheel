#include "lis302dl.h"
#include "usart.h"

void init() {
  initAccelerometer();
  initUSART2(USART2_BAUDRATE_9600);
  enIrqUSART2();
}

int main(void) { 
  init();

  int8_t accelerometerData[3];
  
  getDataFromAccelerometer(accelerometerData);
 
  //TODO prepare accelerometer data to send via bluetooth

  return 0; 
}
