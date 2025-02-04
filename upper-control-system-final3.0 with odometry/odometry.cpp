#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "odometry.h"

float distance[2]={0.0,0.0};
float position[3]={0.0,0.0,0.0};
long int count[2]={0},prev[2]={0};
float Dc=0;
struct encoder *encoderA,*encoderB;
void initOdometry(struct encoder *x,struct encoder *y)
{
	encoderA=x;
	encoderB=y;		
}
float getX(float DC)
{
	position[X] = position[X]+DC*cos((position[Phi]));//(pi*position[Phi])/180.0);
	return position[X];
}
float getY(float DC)
{
	position[Y] = position[Y]+DC*sin((position[Phi]));
	return position[Y];
}
float getPhi()
{
	position[Phi] = position[Phi]+(distance[A]-distance[B])/(DistanceBetweenWheels);
	return position[Phi];
}
float getDc()
{
	distance[A]=(2*pi*RADIUSA*GetTicks(A))/NumberOfTicksPerRevolution;
	distance[B]=(2*pi*RADIUSB*GetTicks(B))/NumberOfTicksPerRevolution;
	return 	(distance[A]+distance[B])/2.0;
}
long int GetTicks(int i)
{
	int x,y;
	if(i==A)
	{
		count[A]= encoderA->value;
		x= count[A]-prev[A];
		prev[A]=count[A];
		return x;
	}
	if(i==B)
	{
		count[B]=encoderB->value;
		x=(count[B]-prev[B]);
		prev[B]=count[B];
		return x;
	}

}
