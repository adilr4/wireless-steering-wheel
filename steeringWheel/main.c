#include "lis302dl.h"

int main(void) { 
  int8_t accelerometerData[3];
  
  initAccelerometer(); 
  
  getDataFromAccelerometer(accelerometerData);
 
  //TODO prepare accelerometer data to send via bluetooth

  return 0; 
}
