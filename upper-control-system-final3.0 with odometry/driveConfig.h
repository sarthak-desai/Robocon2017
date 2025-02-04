#ifndef driveConfig
#define driveConfig

/*Buttons config*/
#define powerOffButton 4
#define headingRefButton 0
#define ps2InputLED 25
#define headingLED 29
#define slowLoopLED 27
#define miscLED 28
extern bool ps2Ready;
extern bool imuReady;
extern bool powerOffPressed;


#define heightMotorPin1 23
#define heightMotorPin2 24
#define heightMotorPWM 	26

extern bool rotatePressed;
extern bool rotateDirection;

extern int mode;
extern int curMode;
extern int preMode;

extern float desiredJunction;
extern float lastJunction;

enum{clk,antiClk};

#define HEADING_TOL 2
#define HEIGHT_TOLERANCE 5

enum {left, right}; //wheel
enum {headingControl,lineControl_fw,lineControl_bw,heightControl};

struct differentialState {int leftRPM; int rightRPM;};

//Bot specifications
#define wheelRadius 5.0
#define wheelCircumference (2 * PI * wheelRadius)
#define L					47.5
#define wRotate 				1.0

//Motion constraints specifications
#define maxRPM			252	//in rpm
#define maxVelocity		300 //in cm/s

//Frequency specifications
#define PIDfrequency	40

#endif
