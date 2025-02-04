#ifndef ODOMETRY
#define ODOMETRY
#include "encoder1.h"

#define DistanceBetweenWheels 96.0
#define RADIUSA 5.0
#define RADIUSB 5.0
#define pi 3.14159
#define NumberOfTicksPerRevolution 3995.0
enum{X,Y,Phi};
enum{A,B};
float getX(float DC);
float getY(float DC);
float getPhi();
float getDc();
long int GetTicks(int i);
void initOdometry(struct encoder *x,struct encoder *y);


#endif 
