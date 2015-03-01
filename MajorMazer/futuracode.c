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


//CONSTANTS
int32_t hasLeftWall = 727;
int32_t hasRightWall = 746;  
int32_t leftMiddleValue = 1227;
int rightMiddleValue = 2306;
double P = .01;
double D = .005;
int leftBaseSpeed = 100;
int rightBaseSpeed = 100;
//NOT CONSTANTS
int32_t errorP = 0;
int32_t errorD = 0;
double totalError = 0;
int32_t oldErrorP = 0;
//STATE VARIABLES
bool straightRun = true;
bool stopped = false;
bool turning = false;
//BRANCH VARIABLES
bool stopped_bn_straightRun = false;
bool stopped_bn_turning = true;
int numTurns = 0;
//TIMING VARIABLE
int waitTime = 0;



///////////////////////////
///////Our code!///////////
///////////////////////////

///////////////////////////
//Basic movements//////////
///////////////////////////
void stop()
{
		setLeftPwm(0);
		setRightPwm(0);
}

void turnRight()
{
	displayMatrix("RIGT");
	setLeftPwm(60);
	setRightPwm(-60);
}

void turn90Right()
{
	int left = getLeftEncCount();
	int right = getRightEncCount();  
	//printf("lenc %d renc %d\r\n",left, right);
	while(!(getLeftEncCount()-left> 1100 && getRightEncCount()-right< -800))
	{
		turnRight();
	}
}
void turn180Right()
{
	int left = getLeftEncCount();
	int right = getRightEncCount();  
	//printf("lenc %d renc %d\r\n",left, right);
	while(!(getLeftEncCount()-left> 2700 && getRightEncCount()-right< -2100))
	{
		turnRight();
	}
}

///////////////////////////
///////////////////////////
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
	setLeftPwm(leftBaseSpeed - totalError);
	setRightPwm(rightBaseSpeed + totalError);    
}

///////////////////////////
///////////////////////////
///////////////////////////

void systick(void) {
	readSensor();
	//the following conditions shall be considered as states
	if(straightRun && !stopped && !turning)
	{
			PID();
			displayMatrix("PID");
			printf("PID\n");
			if(LFSensor> 600 || RFSensor> 1600)
			{
				//branch to stopped
				straightRun = false;
				stopped = true;
				turning = false;
				stopped_bn_straightRun = false;
				stopped_bn_turning = true;
			}
			
			//reset numTurns
			numTurns = 0;
	}
	else if(!straightRun && stopped && !turning)
	{
		stop();
		printf("STOP\n");
		waitTime++;	//timing variable so that it stops for a while before turning.
		if(!stopped_bn_straightRun && stopped_bn_turning) 		//branch to turn
		{
			if(waitTime >= 250)
			{
				straightRun = false;
				stopped = false;
				turning = true;
				waitTime = 0;
			}
		}
		else if (stopped_bn_straightRun && !stopped_bn_turning)		//branch to straightRun
		{
			if(waitTime >= 250)
			{
				straightRun = true;
				stopped = false;
				turning = false;
				waitTime = 0;
			}
		}
	}
	else if(!straightRun && !stopped && turning)
	{
		printf("TURN\n");
			turn180Right();
		//no more turning;
		//branch back to straightRuns
			straightRun = false;
			stopped = true;
			turning = false;
			stopped_bn_straightRun = true;
			stopped_bn_turning = false;
	}
}

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
	
	displayMatrix("PID");
	/*
	//shortBeep(2000, 8000);
	while(1) {
		
		readGyro();
		readVolMeter();
		printf("LF %d RF %d DL %d DR %d aSpeed %d angle %d voltage %d lenc %d renc %d\r\n", LFSensor, RFSensor, DLSensor, DRSensor, aSpeed, angle, voltage, getLeftEncCount(), getRightEncCount());
		
		
		setLeftPwm(100);
		setRightPwm(100);
		delay_ms(1000);
	}
	*/
}
