#include "stm32f4xx.h"

#include "delay.h"

#include "led.h"

#include "button.h"

#include <stdio.h>

#include "usart.h"

#include "SPI.h"

#include "matrixDisplay.h"

#include "pwm.h"

#include "encoder.h"

#include "buzzer.h"

#include "main.h"

#include "sensor_Function.h"

#include "adc.h"

#include <string.h>



///////////////////////////

//Globals//////////////////

///////////////////////////



//CONSTANTS

int32_t hasLeftWall = 800;

int32_t hasRightWall = 1300;  

int32_t hasFrontWallLeft = 872; 

int32_t hasFrontWallRight = 1600; 

int32_t leftMiddleValue = 1445;

int32_t rightMiddleValue = 1880;

double P = 0.01;

double D = 0.01;

int leftBaseSpeed = 100;

int rightBaseSpeed = 100;

volatile int left_enc;

volatile int right_enc;

volatile int gyro;

int forward_left_pwm=20;

int forward_right_pwm=20;

//NOT CONSTANTS

int32_t errorP = 0;

int32_t errorD = 0;

double totalError = 0;

int32_t oldErrorP = 0;

//ENCODER CONSTANTS VARIABLES

int rightEncoderDeltaCell = 5000;

int leftEncoderDeltaCell = 4400;

int targetLeft = 0;

int targetRight = 0;

const int LEFT = 1;

const int RIGHT = -1;

int turning = 0;







///////////////////////////

///////////////////////////

///////////////////////////





///////////////////////////

//Basic movements//////////

///////////////////////////

void PID(void) 

{

	

	if((DLSensor > hasLeftWall && DRSensor > hasRightWall))//has both walls

	{

	//	printf("Both\r\n");

		//ccw direction is positive

		errorP = DRSensor - DLSensor - (rightMiddleValue - leftMiddleValue);

		//rightMiddleValue - leftMiddleValue is the offset between left and right sensor when mouse in the middle of cell

			errorD = errorP - oldErrorP;

		//printf("%d", errorP);

	}        

	else if((DLSensor > hasLeftWall))//only has left wall

	{

	//	printf("L\r\n");

			errorP = 2 * (leftMiddleValue - DLSensor);

			errorD = errorP - oldErrorP;

	}

	else if((DRSensor > hasRightWall))//only has right wall

	{

		//printf("R\r\n");

			errorP = 2 * (DRSensor - rightMiddleValue);

			errorD = errorP - oldErrorP;

	}



	else if((DLSensor < hasLeftWall && DRSensor <hasRightWall))//no wall, use encoder or gyro

	{

	//printf("None\r\n");

			errorP = 0;//(leftEncoder – rightEncoder*1005/1000)*3;

			errorD = 0;

	}

	forward_left_pwm = targetLeft;

	forward_right_pwm = targetRight;

	if (turning == 0) {

		totalError = P * errorP + D * errorD;

		oldErrorP = errorP;

		forward_left_pwm -= totalError;

		forward_right_pwm += totalError;

	}

	setLeftPwm(forward_left_pwm);

	setRightPwm(forward_right_pwm);

		/*MIGHT USE FOR STOPPPING

	if(LFSensor >= hasFrontWallLeft || RFSensor >= hasFrontWallRight)

	{

		forward_left_pwm = 0;

		forward_right_pwm = 0;

	}

	*/

}

void stop(int time)

{

	displayMatrix("STOP");

	targetLeft = 0;

	targetRight = 0;

	delay_ms(time);

}



void goForward(int time, int left_pwm_speed,int right_pwm_speed) 

{

	displayMatrix("FWD");

  setLeftPwm(forward_left_pwm);

	setRightPwm(forward_right_pwm);

	delay_ms(time);

}



void goForwardandStop(int time, int left_pwm_speed,int right_pwm_speed) 

{

	//printf("forward_left_pwm %d ,forward_right_pwm %d \r\n",forward_left_pwm,forward_right_pwm);



	displayMatrix("FWD");

	targetLeft = left_pwm_speed;

	targetRight = right_pwm_speed;

  //setLeftPwm(forward_left_pwm);

	//setRightPwm(forward_right_pwm);

	delay_ms(time);

}



void forwardDistance(int distance, int left_speed, int right_speed, bool coast) {

	int curEnc = left_enc;

	while (left_enc - curEnc < distance) {

		displayMatrix("FWD");

		targetLeft = left_speed;

		targetRight = right_speed;

	}

	if (!coast) {

		targetLeft = 0;

		targetRight = 0;

	}

}



void turnRight(int time, int left_pwm_speed,int right_pwm_speed) 

{

	displayMatrix("RIGT");

	turning = 1;

	setLeftPwm(left_pwm_speed);

	setRightPwm(-right_pwm_speed);

	turning = 0;

	/*targetLeft = left_pwm_speed;

	targetRight = -right_pwm_speed;*/

	delay_ms(time);

}

void turnLeft(int time, int left_pwm_speed,int right_pwm_speed) 

{

	displayMatrix("LEFT");

	/*setLeftPwm(-left_pwm_speed);

	setRightPwm(right_pwm_speed);*/

	targetLeft = left_pwm_speed;

	targetRight = -right_pwm_speed;

	delay_ms(time);

}



void turnDegrees(int degrees, int direction) {

	int curAng;

	curAng = angle;

	while (angle-curAng < degrees) {

		printf("angle: %d\tcurAngle: %d\tangle-curAngle: %d\r\n", angle, curAng, angle-curAng); 

		targetLeft = -50*direction;

		targetRight = 50*direction;

	}

	targetLeft = 0;

	targetRight = 0;

}



void turn90Left(int left_pwm_speed,int right_pwm_speed)  

{

	/*resetRightEncCount();

  resetLeftEncCount();

	left_enc = 0;

	right_enc = 0;

	//go foward one cell, then turn left

	while(left_enc < leftEncoderDeltaCell || right_enc < rightEncoderDeltaCell)

	{

		goForward(0,left_pwm_speed, right_pwm_speed);

	}*/

	

	

	//not exactly sure why, but need to reset AND set the global value to zero

	displayMatrix("LEFT");

	resetRightEncCount();

  resetLeftEncCount();

	left_enc = 0;

	right_enc = 0;

	turning = 1;

	while(left_enc > -1100 || right_enc < 1100)

	{

		turnLeft(0,60,60);

		if (left_enc > -1100) {

			displayMatrix("TITS");

		}

		else {

			displayMatrix("ASS");

		}

		//printf("lenc %d renc %d\r\n", left_enc, right_enc);

	}

  //reset the global variables.	

	displayMatrix("LEFT");

	turning = 0;

	targetLeft = 0;

	targetRight = 0;

}



void turn90Right(int left_pwm_speed,int right_pwm_speed)  

{

	/*resetRightEncCount();

  resetLeftEncCount();

	left_enc = 0;

	right_enc = 0;

	//go foward one cell, then turn left

	while(left_enc < leftEncoderDeltaCell || right_enc < rightEncoderDeltaCell)

	{

		goForward(0,left_pwm_speed,right_pwm_speed);

				printf("lenc %d renc %d\r\n", left_enc, right_enc);

	}

	

	stop(1000);*/

	

	//not exactly sure why, but need to reset AND set the global value to zero

	displayMatrix("RIGT");

	resetRightEncCount();

  resetLeftEncCount();

	left_enc = 0;

	right_enc = 0;

	//printf("\n");

	turning = 1;

	while(left_enc< 1200 || right_enc> -1200)

	{

		turnRight(0,60,60);

		//printf("lenc %d renc %d\r\n", left_enc, right_enc);

	}



  //reset the global variables.

	turning = 0;	

	targetLeft = 0;

	targetRight = 0;



}

///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////



void turnRightAngleRight(int leftSpeed, int rightSpeed) {

	int curRenc = right_enc;

	int curLenc = left_enc;

	turning = 1;

	while (left_enc - curLenc < 1150 || curRenc - right_enc < 1150) {

		displayMatrix("RIGT");

		targetLeft = leftSpeed;

		targetRight = -rightSpeed;

	}

	turning = 0;

	targetLeft = 0;

	targetRight = 0;

}



void turnRightAngleLeft(int leftSpeed, int rightSpeed) {

	int curRenc = right_enc;

	int curLenc = left_enc;

	turning = 1;

	while (right_enc - curRenc < 1150 || curLenc - left_enc < 1150) {

		displayMatrix("LEFT");

		targetLeft = -leftSpeed;

		targetRight = rightSpeed;

	}

	turning = 0;

	targetLeft = 0;

	targetRight = 0;

}



void systick(void) {

	

	readGyro();

	readVolMeter();

	readSensor();

	PID();

	left_enc = getLeftEncCount();

	right_enc = getRightEncCount();  

		//gyro = angle;

}

///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////

void button1_interrupt(void) {

}





void button2_interrupt(void) {

}



int main(void) {

	int runSpeed = 50;

	Systick_Configuration();

	LED_Configuration();

	button_Configuration();

	usart1_Configuration(9600);

	SPI_Configuration();

  TIM4_PWM_Init();

	Encoder_Configration();

	buzzer_Configuration();

	ADC_Config();

	

	

	//shortBeep(2000, 8000);

	stop(1000);

	/*forwardDistance(leftEncoderDeltaCell, 100, 100, false);

	forwardDistance(leftEncoderDeltaCell, 100, 100, true);

	forwardDistance(leftEncoderDeltaCell, 100, 100, false);*/

	//turnDegrees(90, 1);

	//goForward(0,0,0);

	//shortBeep(2000, 8000);

	//turn90Right(0,100,100);

	/*while(1) {

		//readGyro();

		//readVolMeter();

  	//readSensor();

		forwardDistance(leftEncoderDeltaCell, 50, 50, true);

		forwardDistance(leftEncoderDeltaCell, 50, 50, true);

		forwardDistance(leftEncoderDeltaCell, 50, 50, true);

		forwardDistance(leftEncoderDeltaCell, 50, 50, false);

		stop(1000);

		turnRightAngleRight(50, 50);

		stop(1000);

		turnRightAngleRight(100, 100);

		stop(1000);

		forwardDistance(leftEncoderDeltaCell, 50, 50, true);

		forwardDistance(leftEncoderDeltaCell, 50, 50, true);

		forwardDistance(leftEncoderDeltaCell, 50, 50, true);

		forwardDistance(leftEncoderDeltaCell, 50, 50, false);

		stop(1000);

		turnRightAngleLeft(50, 50);

		stop(1000);

		turnRightAngleLeft(100, 100);

		stop(1000);

	//printf("LF %d RF %d DL %d DR %d\r\n", LFSensor, RFSensor, DLSensor, DRSensor);

		printf("LF %d RF %d DL %d DR %d aSpeed %d angle %d voltage %d lenc %d renc %d\r\n", LFSensor, RFSensor, DLSensor, DRSensor, aSpeed, angle, voltage, getLeftEncCount(), getRightEncCount());

		//setLeftPwm(100);

		//setRightPwm(100);

	}*/

	if (1) {

	forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);

	forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);

	forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);

	forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);

	forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);

	stop(1000);

	turnRightAngleLeft(runSpeed, runSpeed);

	stop(1000);

	forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);

	stop(1000);

	turnRightAngleLeft(runSpeed, runSpeed);

	stop(1000);

	forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);

	forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);

	stop(1000);

	turnRightAngleLeft(runSpeed, runSpeed);

	stop(1000);

	forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);

	stop(1000);

	turnRightAngleLeft(runSpeed, runSpeed);

	stop(1000);

	forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);

	stop(1000);

	turnRightAngleRight(runSpeed, runSpeed);

	stop(1000);

	forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);

	stop(1000);

	turnRightAngleRight(runSpeed, runSpeed);

	stop(1000);

	forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);

	stop(1000);

	turnRightAngleLeft(runSpeed, runSpeed);

	stop(1000);

	forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);

	stop(10000);

	}

	

	/*while(1) {

		printf("LF %d RF %d DL %d DR %d aSpeed %d angle %d voltage %d lenc %d renc %d\r\n", LFSensor, RFSensor, DLSensor, DRSensor, aSpeed, angle, voltage, getLeftEncCount(), getRightEncCount());

	}*/

}

