#include "encoder.h"
extern ENCODER *enc1;
void updateEncoder()
{
	if(digitalRead(enc1->pinb))
	{
		enc1->value=enc1->value+1;
	}
	else
	{
		enc1->value=enc1->value-1;
	}	
}
void resetEnc(){
enc1->value=0;
}
long int getHeight()
{
return enc1->value;
}
ENCODER* setupEncoder(int pin_1,int pin_2)
{
	ENCODER *newencoder;
	newencoder=(ENCODER*)malloc(sizeof(ENCODER));
	newencoder->pina = pin_1;
	newencoder->pinb = pin_2;
	newencoder->value=0;
	pinMode(pin_1,INPUT);
	//pullUpDnControl(pin_1,PUD_DOWN);
	pinMode(pin_2,INPUT);
	//pullUpDnControl();
	wiringPiISR(pin_1,INT_EDGE_RISING,updateEncoder);
	return newencoder; 
}
/*int main()
{
	if(wiringPiSetup()<0)
	{
	printf("setup failed");
	}
	enc1=setupEncoder(25,28);
	while(1){
		//printf("%d\n",enc1->value);
	//sleep(1);
	}
	return 0;
}*/
