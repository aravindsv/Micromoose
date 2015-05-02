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
int32_t hasLeftWall = 727;
int32_t hasRightWall = 746;  
int32_t hasFrontWall = 746;  
int32_t leftMiddleValue = 1227;
int rightMiddleValue = 2306;
double P = .01;
double D = .005;
int leftBaseSpeed = 100;
int rightBaseSpeed = 100;
volatile int left_enc;
volatile int right_enc;
volatile int gyro;
int forward_left_pwm=100;
int forward_right_pwm=100;
//NOT CONSTANTS
int32_t errorP = 0;
int32_t errorD = 0;
double totalError = 0;
int32_t oldErrorP = 0;
//ENCODER CONSTANTS VARIABLES
int rightEncoderDeltaCell = 5000;
int leftEncoderDeltaCell = 4000;

///////////////////////////
///////////////////////////
///////////////////////////


///////////////////////////
//Basic movements//////////
///////////////////////////
void PID(void) 
{
	readSensor();
	if((DLSensor > hasLeftWall && DRSensor > hasRightWall))//has both walls
	{  //ccw direction is positive
			errorP = DRSensor - DLSensor - 0;
		//63 is the offset between left and right sensor when mouse in the middle of cell
			errorD = errorP - oldErrorP;
		//printf("%d", errorP);
	}        
	else if((DLSensor > hasLeftWall))//only has left wall
	{
			errorP = 2 * (leftMiddleValue - DLSensor);
			errorD = errorP - oldErrorP;
	}
	else if((DRSensor > hasRightWall))//only has right wall
	{
			errorP = 2 * (DRSensor - rightMiddleValue);
			errorD = errorP - oldErrorP;
	}
	else if((DLSensor < hasLeftWall && DRSensor <hasRightWall))//no wall, use encoder or gyro
	{
			errorP = 0;//(leftEncoder – rightEncoder*1005/1000)*3;
			errorD = 0;
	}
	totalError = P * errorP + D * errorD;
	oldErrorP = errorP;
	forward_left_pwm = leftBaseSpeed - totalError;
	forward_right_pwm = rightBaseSpeed + totalError;
	//printf("forward_left_pwm %d\t",forward_left_pwm);
	//printf("forward_right_pwm %d\t\r\n",forward_right_pwm);
	//printf("*************\r\n");
//	setLeftPwm(leftBaseSpeed - totalError);
//	setRightPwm(rightBaseSpeed + totalError);    
}
void stop(int time)
{
	displayMatrix("STOP");
	setLeftPwm(0);
	setRightPwm(0);
	delay_ms(time);
}

void goForward(int time, int left_pwm_speed,int right_pwm_speed) 
{
	
	displayMatrix("FWD");
	printf("\nHELLO\n");
		printf("gforward_left_pwm %d\t",forward_left_pwm);
	printf("gforward_right_pwm %d\t\r\n",forward_right_pwm);
	setLeftPwm(forward_left_pwm);
	setRightPwm(forward_right_pwm);
	delay_ms(time);
}

void forwardDistance(int distance, int left_speed, int right_speed, bool coast) {
	int curEnc = left_enc;
	char displayStr[4];
	snprintf(displayStr, 4, "%d", distance);
	while (left_enc - curEnc < distance) {
		setLeftPwm(left_speed);
		setRightPwm(-right_speed);
	}
	if (!coast) {
		setLeftPwm(0);
		setRightPwm(0);
	}
}

void turnRight(int time, int left_pwm_speed,int right_pwm_speed) 
{
	displayMatrix("RIGT");
	setLeftPwm(left_pwm_speed);
	setRightPwm(-right_pwm_speed);
	delay_ms(time);
}
void turnLeft(int time, int left_pwm_speed,int right_pwm_speed) 
{
	displayMatrix("LEFT");
	setLeftPwm(-left_pwm_speed);
	setRightPwm(right_pwm_speed);
	delay_ms(time);
}

void turnDegrees(int degrees, int direction) {
	int curAng;
	readGyro();
	curAng = angle;
	while (angle-curAng < 300) {
		readGyro();
		printf("angle: %d\tcurAngle: %d\tangle-curAngle: %d\r\n", angle, curAng, angle-curAng); 
		setLeftPwm(-50);
		setRightPwm(-50);
	}
	setLeftPwm(0);
	setRightPwm(0);
}

void turn90Left(int time, int left_pwm_speed,int right_pwm_speed)  
{
	resetRightEncCount();
  resetLeftEncCount();
	left_enc = 0;
	right_enc = 0;
	//go foward one cell, then turn left
	while(left_enc < leftEncoderDeltaCell || right_enc < rightEncoderDeltaCell)
	{
		goForward(0,left_pwm_speed,right_pwm_speed);
	}
	
	stop(1000);
	
	//not exactly sure why, but need to reset AND set the global value to zero
	resetRightEncCount();
  resetLeftEncCount();
	left_enc = 0;
	right_enc = 0;
	printf("\n");
	while(left_enc> -1100 || right_enc< 1100)
	{
		turnLeft(0,60,60);
		printf("lenc %d renc %d\r\n", left_enc, right_enc);
	}
  //reset the global variables.	
	stop(1000);
}

void turn90Right(int time, int left_pwm_speed,int right_pwm_speed)  
{
	resetRightEncCount();
  resetLeftEncCount();
	left_enc = 0;
	right_enc = 0;
	//go foward one cell, then turn left
	while(left_enc < leftEncoderDeltaCell || right_enc < rightEncoderDeltaCell)
	{
		goForward(0,left_pwm_speed,right_pwm_speed);
				printf("lenc %d renc %d\r\n", left_enc, right_enc);
	}
	
	stop(1000);
	
	//not exactly sure why, but need to reset AND set the global value to zero
	resetRightEncCount();
  resetLeftEncCount();
	left_enc = 0;
	right_enc = 0;
	printf("\n");
	while(left_enc< 1200 || right_enc> -1200)
	{
		turnRight(0,60,60);
		printf("lenc %d renc %d\r\n", left_enc, right_enc);
	}

  //reset the global variables.	
	stop(1000);

}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void systick(void) {
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

	/*forwardDistance(leftEncoderDeltaCell, 100, 100, false);
	forwardDistance(leftEncoderDeltaCell, 100, 100, true);
	forwardDistance(leftEncoderDeltaCell, 100, 100, false);*/
	//turnDegrees(90, 1);
	//goForward(0,0,0);
	while(1) {
		printf("Qhile loop\r\n");
		//goForward(0,0,0);	
		
		readSensor();
		readGyro();
		readVolMeter();
		printf("LF %d RF %d DL %d DR %d aSpeed %d angle %d voltage %d lenc %d renc %d\r\n", LFSensor, RFSensor, DLSensor, DRSensor, aSpeed, angle, voltage, getLeftEncCount(), getRightEncCount());
		displayMatrix("mous");
		
		//setLeftPwm(100);
		//setRightPwm(100);
		delay_ms(1000);
	}

}
