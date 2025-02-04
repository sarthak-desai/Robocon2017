// Compile with -lpthread -lwiringPi
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <softPwm.h>
#include <stdlib.h>
#include <fstream>
#include <istream>
#include "encoder1.h"
#include "odometry.h"
#include "driveConfig.h"
#include "userlib_interrupt.h"
#include "minimu9.h"
#include "ps2USB.h"
#include "ps2_2k17.h"
#include "timerInterrupt.h"
#include "controlmath.h"
#include "pidController.h"
#include "LSA08.h"
#include "mechanismConfig.h"

//Interrupt function definitions
bool ps2Ready = false;
bool imuReady = false;
bool powerOffPressed = false;
void slowTimerHandler();
void ps2Activated();
void ps2Deactivated();
void imuActivated();
void imuDeactivated();

struct unicycleState stopUniCycleState;
struct differentialState stopState;
struct differentialState curDiffState;
struct differentialState desiredDiffState;
struct lineSensor ls1,ls2;
struct encoder *encoder1;
struct encoder *encoder2;
float desiredHeading = 0.0;
float headingCorrection = 0;
float headingError = 0;
float prev_desiredPhi = 0;
float prev_vy = 0;
int desiredHeight = 0;
int prev_right_analog_stick = 128, right_analog_stick = 128;

bool forward = true, reverse = false, rotateWasPressed = false;
float dc=0.0;
extern int shoot,load,planeAngle;
int maxPWM = 0, minPWM = 0;

float ls1_error = 0, ls1_prev_error = 0;
float ls2_error = 0, ls2_prev_error = 0;

int rpiPort;

void reset() {
	printf("RESET\n");
	resetRefHeading();
	resetPIDvar(headingControl);
	resetPIDvar(lineControl_fw);
	resetPIDvar(lineControl_bw);
	ls1_error = 0;
	ls1_prev_error = 0;
	ls2_error = 0;
	ls2_prev_error = 0;
	headingCorrection = 0;
	prev_desiredPhi = 0;
	prev_vy = 0;
	lastJunction = 0;
	desiredJunction = 0;
	forward = true;
	reverse = false;
	rotateWasPressed = false;
	shoot = 0;
	load = 0;
	planeAngle = 0;
	//resetEnc();
	transmitMechanismControl(shoot,load,planeAngle,initialRPM,initialPos);
}

char encodeByte(int rpm) {
    if(rpm > maxRPM) {
        printf("Warning: Trying to send Velocity greater than 180. Limit : 180. Sending 180.\r\n");
        rpm = 252; //must be even (maxRPM)
    } else if(rpm < -(maxRPM + 1)) {
        printf("Warning: Trying to send Velocity lesser than -181. Limit : -181. Sending -181.\r\n");
        rpm = -251; // must be odd
    }
    if(rpm < 0) {
        rpm = -rpm;
        return (rpm | 0x01);
    } else {
        rpm = (rpm & 0xFE);
        return rpm == 0x0A ? 0x0C : rpm;
    }
}

void transmitDiffState(struct differentialState desiredDiffState) {
    serialPutchar(rpiPort, 0x0A);
    serialPutchar(rpiPort, encodeByte(desiredDiffState.leftRPM));
    serialPutchar(rpiPort, encodeByte(desiredDiffState.rightRPM));
//    printf("%d %d %f \n",desiredDiffState.leftRPM,desiredDiffState.rightRPM,getHeading());
}


//Function to read linesensors and preprocess it.
void lineFeedback(void) {
//	if(digitalRead(ls1.NANDoutPin)) {
		ls1_error = readLineSensor(ls1);
//	}
	if(ls1_error == 255) {
		if(ls1_prev_error < 0) {
			ls1_error = -50;
		} else if(ls1_prev_error > 0) {
			ls1_error = 50;
		} else {
			ls1_error = 0;
		}
	}
	ls1_prev_error = ls1_error;
	usleep(1000);	//readings skew if this is removed. Need further study : Aniket,20 October,2016
//	if(digitalRead(ls2.NANDoutPin)) {
		ls2_error = readLineSensor(ls2);
//	}
	if(ls2_error == 255) {
		if(ls2_prev_error < 0) {
			ls2_error = -50;
		} else if(ls2_prev_error > 0) {
			ls2_error = 50;
		} else {
			ls2_error = 0;
		}
	}
	ls2_prev_error = ls2_error;
}

//Map velocity according to percent path
float velocityMap(void) {
	float velocity = 0;
	int percentPath = desiredJunction - lastJunction;
	switch(abs(percentPath)) {
		case 0:
			velocity = 0;
			break;
		case 1:
			velocity = maxVelocity * 0.3;
			break;
		case 2:
			velocity = maxVelocity * 0.6;
			break;
		case 3:
			velocity = maxVelocity * 0.9;
			break;
		case 4:
			velocity = maxVelocity;
			break;
		case 5:
			velocity = maxVelocity;
			break;
	}
	if (percentPath > 0) {
		forward = true;
		reverse = false;
	} else if(percentPath < 0) {
		forward = false;
		reverse = true;
		velocity = -velocity;
	} else {
		forward = false;
		reverse = false;
	}
	return velocity;
}

//Function for manual driving with heading control
struct unicycleState getDesiredUnicycleState_manual(void) {
	struct unicycleState desiredState;
	rotateCheck();
	if(rotatePressed) {
	        desiredState = rotateBot();
			rotateWasPressed = true;
	} else {
		float vy, vx;
		desiredState.vy = (128 - ps2_getY()) * maxVelocity / 128;
		desiredState.vx = 0;
		if(rotateWasPressed == true) {
			if(desiredState.vy == 0) {
				return stopUniCycleState;
			} else {
				headingOffset = getHeading();
				rotateWasPressed = false;
			}
		}
		float desiredPhi;
		if(desiredState.vy == 0 && desiredState.vx == 0) {
			desiredPhi = prev_desiredPhi;
		} else {
			desiredPhi = radianToDegree((PI/2) - atan2(abs(desiredState.vy), desiredState.vx)); //Absolute of vy so that for negative v the angle does not go -ve and the robot does not turn. We want it to drive backwards
		}
		prev_desiredPhi = desiredPhi;
		float error = desiredPhi - getHeading() + headingOffset;
		error = obtuseAngleFilter(error);
		(fabs(error) > HEADING_TOL) ? desiredState.w = PID(error, headingControl) : desiredState.w = 0;
	}
	return desiredState;
}

//Function for semiautonomous driving with heading control and linesensors
struct unicycleState getDesiredUnicycleState_line(void) {
	struct unicycleState desiredState;
	float vy, vx;
	lineFeedback();
	printf("%f %f ;",ls1_error,ls2_error);
	desiredState.vx = 0;
	desiredState.vy = ((128 - ps2_getY())/128.0) * maxVelocity;
	if(desiredState.vy > 0) {
		desiredState.w = PID(ls1_error, lineControl_fw);
		prev_vy = desiredState.vy;
	} else if(desiredState.vy < 0) {
		desiredState.w = PID(ls2_error, lineControl_bw);
		prev_vy = desiredState.vy;
	} else {
		if(prev_vy >= 0) {
			desiredState.w = PID(ls1_error, lineControl_fw);
		} else {
			desiredState.w = PID(ls2_error, lineControl_bw);
		}
	}
	return desiredState;
}

//Function for autonomous driving with heading control and linesensors
struct unicycleState getDesiredUnicycleState_auto(void) {
	struct unicycleState desiredState;
	float vy, vx;
	lineFeedback();
	desiredState.vx = 0;
	desiredState.vy = velocityMap();
	if(desiredState.vy > 0) {
		desiredState.w = PID(ls1_error, lineControl_fw);
		prev_vy = desiredState.vy;
	} else if(desiredState.vy < 0) {
		desiredState.w = PID(ls2_error, lineControl_bw);
		prev_vy = desiredState.vy;
	} else {
		if(prev_vy >= 0) {
			desiredState.w = PID(ls1_error, lineControl_fw);
		} else {
			desiredState.w = PID(ls2_error, lineControl_bw);
		}
	}
	return desiredState;
}


/**********************************************************************************************/
/**  Function for mode enabled driving  **/
/**********************************************************************************************/

struct unicycleState getDesiredUnicycleState_mode() {
    int botMode = getMode();
    modeChange();
    switch(botMode) {
            case 0: 
	//	    printf("Manual mode :: ");
                    return getDesiredUnicycleState_manual();
                    break;
            case 1: 
    		    printf("Line mode :: ");
		    return getDesiredUnicycleState_line();
                    break;
            case 2: 
		    return stopUniCycleState;
		    printf("Auto mode :: ");
		    return getDesiredUnicycleState_auto();
                    break;
    }
}

void setPWM(float pwm, int i) {
	if (pwm > maxPWM) {
		pwm = maxPWM;
	} 
	if (pwm < -maxPWM) {
		pwm = -maxPWM;
	}
	if(i == heightControl) {
		if(fabs(pwm) < HEIGHT_TOLERANCE) {
			digitalWrite(heightMotorPin1,1);
			digitalWrite(heightMotorPin2,1);
			softPwmWrite(heightMotorPWM,minPWM);
//			printf("%d :: ",minPWM);
		} else if(pwm > 0) {
			if(pwm < minPWM) {
				pwm = minPWM;
			}
			softPwmWrite(heightMotorPWM,pwm);
			digitalWrite(heightMotorPin1,1);
			digitalWrite(heightMotorPin2,0);		
//			printf("%f :: ",pwm);
		} else if (pwm < 0) {
			pwm = -pwm;
			if(pwm < minPWM) {
				pwm = minPWM;
			}
			softPwmWrite(heightMotorPWM,pwm);
			digitalWrite(heightMotorPin1,0);
			digitalWrite(heightMotorPin2,1);
//			printf("%f :: ",pwm);
		}
	}
}

void timerHandler() {
	if(!ps2Ready || !imuReady) {
		transmitDiffState(stopState);
	} else {
/*  Mode control Driving */
//		desiredDiffState = transformUniToDiff(getDesiredUnicycleState_mode());
//		transmitDiffState(desiredDiffState);
//		digitalWrite(miscLED, !digitalRead(miscLED));
//		printf("%d :: %d ;",ps2_getY(),ps2_getX());
//	 	printf("%d %d %f \n",desiredDiffState.leftRPM,desiredDiffState.rightRPM,getHeading());
/*odometry*/
		dc=getDc();
		//printf("%d  %d\n",encoder1->value,encoder2->value);
		printf("x=%f,y=%f,phi=%f \n",getX(dc),getY(dc),180.0*getPhi()/pi);
		
		
		
		
/*  Height control */
//		right_analog_stick = ps2_getRY();
//		if(right_analog_stick == 128 && prev_right_analog_stick != 128) {
//			desiredHeight = enc1->value;
//		}
//		if(right_analog_stick == 128) {
//			digitalWrite(27,0);
//			float heightError = (enc1->value)- desiredHeight ;
//			float out = PID(heightError,heightControl);
//			setPWM(0,heightControl);
//			printf("0 :: ");						
//		} else {
//			digitalWrite(27,1);
//			setPWM((ps2_getRY()-128)/2.0,heightControl);
//			printf("1 :: ");						
//		}
//		prev_right_analog_stick = right_analog_stick;

/* Saving data and status */
//		std::ofstream data_file;
//		data_file.open("data.log",std::ios::app);
//		data_file << desiredDiffState.leftRPM << ";" << desiredDiffState.rightRPM << ";" << getHeading() << ";\n";
//		data_file.close();
	}
//transmitMechanismControl_Packet();
//printf(	"%d %d %d ::%d \n",printer_shoot,printer_load,printer_planeAngle,printer_dataFrame);
}

void init() {
   	stopState.leftRPM = 0;
	stopState.rightRPM = 0;
	stopUniCycleState.vx = 0;
	stopUniCycleState.vy = 0;
	stopUniCycleState.w = 0;

	if(wiringPiSetup() < 0) {
		printf("Error setting up while setting wiringPi\n");
	}

// Button and LED configurations
	pinMode(powerOffButton, INPUT);
	pinMode(headingRefButton, INPUT);
	pinMode(ps2InputLED, OUTPUT);
	pinMode(headingLED, OUTPUT);
	pinMode(slowLoopLED, OUTPUT);
	pinMode(miscLED, OUTPUT);
	if(wiringPiISR(powerOffButton, INT_EDGE_RISING, &powerOff) < 0) {
		printf("Power Off Button interrupt setup error \n");
	}
	if(wiringPiISR(headingRefButton, INT_EDGE_RISING, &reset)) {
		printf("Reset Heading Button interrupt setup error\n");
	}
	digitalWrite(ps2InputLED, LOW);
	digitalWrite(headingLED, LOW);
	digitalWrite(slowLoopLED, LOW);
	digitalWrite(miscLED, LOW);
	
// PS2 and IMU configuration
	enablePS2StatusInterrupt(&ps2Activated, &ps2Deactivated);
	enableIMUStatusInterrupt(&imuActivated, &imuDeactivated);
	enableSlowFuncInterrupt(&slowTimerHandler);
	initPS2();
	initPS2_2k17();		//Robocon 2k17 PS2 configuration
	initIMU();

// Initialize mechanism control system	
	initMechanism();
}

//Interrupt on Junction occurence
void junctionInterrupt1(void) {
}

void junctionInterrupt2(void) {
}

int main() {
	rpiPort = serialOpen("/dev/ttyS0",38400);			/*Serial communication port established*/
	initPIDController(0.05,0.0,0.02,headingControl);		/*PID controller for angular velocity in manual mode*/  //0.05,0.02
	initPIDController(0.013,0.0,0.47,lineControl_fw);	//0.03,0.0,1.2	/*PID controller for angular velocity in linefollow_fw mode*/
	initPIDController(0.015,0.0,0.47,lineControl_bw);	//0.03,0.0,1.2	/*PID controller for angular velocity in linefollow_bw mode*/
	initPIDController(0.16,0.0,0.0,heightControl);
	init();
	
	ls2.address = 1;
	ls2.uartPort = rpiPort;
	ls2.UARTPin = 12;
	ls2.junctionPin = 5;
	ls2.NANDoutPin = 0;

	ls1.address = 2;
	ls1.uartPort = rpiPort;
	ls1.UARTPin = 6;
	ls1.junctionPin = 13;
	ls1.NANDoutPin = 1;

	initLineSensor(ls1, &junctionInterrupt1);
	initLineSensor(ls2, &junctionInterrupt2);
	
//Height control pin init
	encoder1 = setupencoder(21,22);	
	encoder2 = setupencoder(23,24);
	initOdometry(encoder1,encoder2);
	pinMode(heightMotorPin1,OUTPUT);
	pinMode(heightMotorPin2,OUTPUT);
	softPwmCreate(heightMotorPWM,0,255);
	pinMode(27,OUTPUT);
	digitalWrite(27,0);
	digitalWrite(heightMotorPin1,0);
	digitalWrite(heightMotorPin2,0);
	maxPWM = 128;
	minPWM = 5;
	initTimer(1000000/PIDfrequency, &timerHandler);

	while(1) {
		sleep(1);
	}
}
