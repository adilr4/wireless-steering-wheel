#include "lis302dl.h"
#include "usart.h"

const int angleOffset = 9;

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

void getDataFromAngle(float x, float y) {
	if(x > 0) {	//ispravan polozaj
		for (int i = 0; i < 10; ++i)
			if(y < -81 + i * angleOffset) {
				l = i;
				r = 9;
				return;
			}

		for (int i = 0; i < 10; ++i)
			if(y < 9 + i * angleOffset) {
				l = 9;
				r = 9 - i;
				return;
			}
	} else {//neispravan polozaj
		if(y > 70) {
			l = 9;
			r = 0;
		} else if(y < -70) {
			l = 0;
			r = 9;
		} else {
			//stani
		}
	}
}

