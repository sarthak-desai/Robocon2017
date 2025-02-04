#ifndef encoder
#define encoder

#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdlib.h>

struct encoderh {
	int pina;
	int pinb;
	long int value;
};

typedef struct encoderh ENCODER;
//ENCODER *enc1;
void resetEnc();
long int getHeight();
ENCODER *setupEncoder(int pin_1,int pin_2);
void updateEncoder();

#endif
