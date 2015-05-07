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
#include <cstring>


///////////////////////////
//Floodfill//////////////////
///////////////////////////

enum Dir {
    NORTH = 0,
    SOUTH,
    EAST,
    WEST,
    INVALID
};

struct Coord {
 
    int m_distance;
    int m_xPos;
    int m_yPos;
    bool m_isProcessed;
  
};


///////////////////////////
//Globals//////////////////
///////////////////////////


//CONSTANTS
int32_t hasLeftWall = 800;
int32_t hasRightWall = 1300;  
int32_t hasFrontWallLeft = 872; 
int32_t hasFrontWallRight = 1600; 
int32_t leftMiddleValue = 0;
int32_t rightMiddleValue = 0;

double P = 0.01;
double D = 0.01;
int leftBaseSpeed = 100;
int rightBaseSpeed = 100;
volatile int left_enc;
volatile int right_enc;
volatile int cur_angle;
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
int mouseStarted = 0;

///////////////////////////
/////Floodfill//////////////////////
///////////////////////////
bool visitedStart = false;
enum Dir heading;
bool shouldGoForward;
bool shouldGoToCenter = true;
int x = 0;
int y = 0;

const int MAZE_W = 16;
const int MAZE_L = 16;
 
struct Coord c_array[16][16];

const int MoveForward = 0;
const int TurnClockwise = 1;
const int TurnCounterClockwise = 2;
const int TurnAround = 3; 
const unsigned VECTOR_SIZE = 16;

bool frontWall = false;
bool leftWall = true;
bool rightWall = true;
         

uint16_t vectorH[(VECTOR_SIZE * VECTOR_SIZE) / (8*sizeof(uint16_t))]; //Horizontal
uint16_t vectorV[(VECTOR_SIZE * VECTOR_SIZE) / (8*sizeof(uint16_t))]; //Vertical

void setH(unsigned x, unsigned y) {
   if(x < VECTOR_SIZE && y < VECTOR_SIZE)
				vectorH[x] |= 1<<y;
}
void clearH(unsigned x, unsigned y) {
   if(x < VECTOR_SIZE && y < VECTOR_SIZE)
       vectorH[x] &= ~(1<<y);
   }
bool getH(unsigned x, unsigned y) {
   if(x < VECTOR_SIZE && y < VECTOR_SIZE)
        return (vectorH[x] & 1<<y) != 0;
 
        return 0;
}
 
void clearAllH() {
    memset(vectorH, 0, sizeof(vectorH));
}
 
void setAllH() {
		memset(vectorH, ~0, sizeof(vectorH));
}

void setV(unsigned x, unsigned y) {
   if(x < VECTOR_SIZE && y < VECTOR_SIZE)
				vectorV[x] |= 1<<y;
}
 
void clearV(unsigned x, unsigned y) {
   if(x < VECTOR_SIZE && y < VECTOR_SIZE)
       vectorV[x] &= ~(1<<y);
   }
 
bool getV(unsigned x, unsigned y) {
   if(x < VECTOR_SIZE && y < VECTOR_SIZE)
        return (vectorV[x] & 1<<y) != 0;
 
        return 0;
}
 
void clearAllV() {
    memset(vectorV, 0, sizeof(vectorH));
}
 
void setAllV() {
		memset(vectorV, ~0, sizeof(vectorH));
}


struct stack {
   struct Coord s[300];
   int top;
} st;
 
int stfull() {
   if (st.top >= 300 - 1)
      return 1;
   else
      return 0;
}
 
void push(struct Coord item) {
   st.top++;
   st.s[st.top] = item;
}
 
int stempty() {
   if (st.top == -1)
      return 1;
   else
      return 0;
}
 
struct Coord pop() {
   struct Coord item;
   item = st.s[st.top];
   st.top--;
   return (item);
}
 
enum Dir opposite(enum Dir d) {
    switch (d) {
        case NORTH:
            return SOUTH;
        case SOUTH:
            return NORTH;
        case EAST:
            return WEST;
        case WEST:
            return EAST;
        case INVALID:
        default:
            return INVALID;
    }
}
 
enum Dir clockwise(enum Dir d) {
    switch (d) {
        case NORTH:
            return EAST;
        case SOUTH:
            return WEST;
        case EAST:
            return SOUTH;
        case WEST:
            return NORTH;
        case INVALID:
        default:
            return INVALID;
    }
}
 
enum Dir counterClockwise(enum Dir d) {
    switch (d) {
        case NORTH:
            return WEST;
        case SOUTH:
            return EAST;
        case EAST:
            return NORTH;
        case WEST:
            return SOUTH;
        case INVALID:
        default:
            return INVALID;
    }
}



void calculateManahattan(int goalX, int goalY) {
        //set Manhattan Distances
        int xMid = goalX;
        int yMid = goalY;
	
				int i = 0;
				int j = 0;
         
        //Quadrant 1
        for (i = 0; i < 8; i++) {
            for (j = 0; j < 8; j++) {
                c_array[i][j].m_distance = abs(xMid - i) + abs(yMid - j);
            }
        }
        //Quadrant 2
        for (i = 8; i < 16; i++) {
            for (j = 0; j < 8; j++) {
                c_array[i][j].m_distance = abs(xMid - i) - 1 + abs(yMid - j);
            }
        }
        //Quadrant 3
        for (i = 8; i < 16; i++) {
            for (j = 8; j < 16; j++) {
                c_array[i][j].m_distance = abs(xMid - i) + abs(yMid - j) - 2;
            }
        }
         
        //Quadrant 4
        for (i = 0; i < 8; i++) {
            for (j = 8; j < 16; j++) {
                c_array[i][j].m_distance = abs(xMid - i) + abs(yMid - j) - 1;
            }
        }
         
        for (i = 0; i < 16; i++) {
            for (j = 0; j < 16; j++) {
                //std::cout << c_array[i][j].m_distance;
                //std::cout << " ";
            }
            //std::cout << "" << std::endl;
        }
        for (i = 0; i < 16; i++)
        {
            for (j = 0; j < 16; j++)
            {
                c_array[i][j].m_xPos = i;
                c_array[i][j].m_yPos = j;
            }
        }
    }

		bool isWallBetween(struct Coord pos1, struct Coord pos2)
    {
        if (pos1.m_xPos == pos2.m_xPos)
        {
            if (pos1.m_yPos > pos2.m_yPos)
            {
                return getH(pos2.m_xPos, pos2.m_yPos);
            }
            else if (pos1.m_yPos < pos2.m_yPos)
            {
                return getH(pos1.m_xPos, pos1.m_yPos);
            }
            else return false;
        }
        else if (pos2.m_yPos == pos1.m_yPos)
        {
            if (pos1.m_xPos > pos2.m_xPos)
            {
                return getV(pos2.m_xPos, pos2.m_yPos);
            }
            else if (pos1.m_xPos < pos2.m_xPos)
            {
                return getV(pos1.m_xPos, pos1.m_yPos);
            }
            else return false;
        }
        return false;
    }

		void floodFill(int xPos, int yPos) {
			
				//struct stack StackTemp;
			
				int i = 0;
				int shortest = 500;
			
        push(c_array[xPos][yPos]);
        while (!stempty())
        {
            struct Coord cur = pop();
					
            cur.m_isProcessed = true;
            if (cur.m_distance == 0)
            {
                continue;
            }
            shortest = 500;
            for (i = 0; i < 4; i++)
            {
                struct Coord neighbor;
                //Set neighbor to the currect coordinate
                switch (i)
                {
                    case NORTH:
                        if (cur.m_yPos >= 15)
                            continue;
                        neighbor = c_array[cur.m_xPos][cur.m_yPos + 1];
                        break;
                    case SOUTH:
                        if (cur.m_yPos <= 0)
                            continue;
                        neighbor = c_array[cur.m_xPos][cur.m_yPos - 1];
                        break;
                    case EAST:
                        if (cur.m_xPos >= 15)
                            continue;
                        neighbor = c_array[cur.m_xPos + 1][cur.m_yPos];
                        break;
                    case WEST:
                        if (cur.m_xPos <= 0)
                            continue;
                        neighbor = c_array[cur.m_xPos - 1][cur.m_yPos];
                        break;
                }
                //Check if there is a wall between cur and neighbor
                if (!isWallBetween(cur, neighbor))
                {
                    if (neighbor.m_distance < shortest)
                    {
                        shortest = neighbor.m_distance;
                    }
                    if (!neighbor.m_isProcessed)
                    {
                        neighbor.m_isProcessed = true;
                    }
                }
            }
            if (shortest == 500)
            {
                continue;
            }
            if (cur.m_distance == (shortest + 1))
            {
                continue;
            }
            else
            {
                c_array[cur.m_xPos][cur.m_yPos].m_distance = shortest + 1;
            }
            for (i = 0; i < 4; i++)
            {
                struct Coord neighbor;
                //Set neighbor to the currect coordinate
                switch (i)
                {
                    case NORTH: 
                        if (cur.m_yPos >= 15)
                            continue;
                        neighbor = c_array[cur.m_xPos][cur.m_yPos + 1];
                        break;
                    case SOUTH:
                        if (cur.m_yPos <= 0)
                            continue;
                        neighbor = c_array[cur.m_xPos][cur.m_yPos - 1];
                        break;
                    case EAST:
                        if (cur.m_xPos >= 15)
                            continue;
                        neighbor = c_array[cur.m_xPos + 1][cur.m_yPos];
                        break;
                    case WEST:
                        if (cur.m_xPos <= 0)
                            continue;
                        neighbor = c_array[cur.m_xPos - 1][cur.m_yPos];
                        break;
                }
                //Check if there is a wall between cur and neighbor
                if (!isWallBetween(cur, neighbor))
                {
                    push(neighbor);
                }
            }
        }
    }

		
		int nextMovement() {
         
         

				int i = 0;
				int j = 0;
			
			
			  int frontX = 0;
        int frontY = 0;
        int rightX = 0;
        int rightY = 0;
        int leftX = 0;
        int leftY = 0;
         
        // If we hit the start of the maze a second time, then
        // we couldn't find the center and never will...
        if (x == 0 && y == 0) {
            if (visitedStart) {
//                std::cout << "Time to go back to the center" << std::endl;
//                calculateManahattan(0, 0);
//                floodFill(x, y);
//                heading = opposite(heading);
//                return TurnAround;
            }
            else {
                visitedStart = true;
            }
        }
         
         
        //Set wall positions
        //LEFTWALL
        if (leftWall) {
            if (heading == NORTH) {
                if (x != 0){
                    setV(x - 1, y);
                }
            }
            else if (heading == SOUTH) {
                setV(x, y);
                 
            }
            else if (heading == WEST) {
                if (y != 0){
                    setH(x, y - 1);
                }
                 
            }
            else if (heading == EAST) {
                setH(x, y);
            }
        }
         
        //RIGHTWALL
        if (rightWall) {
            if (heading == NORTH) {
                setV(x, y);
                 
            }
            else if (heading == SOUTH) {
                if (x != 0){
                    setV(x - 1, y);
                }
                 
            }
            else if (heading == WEST) {
                setH(x, y);
                 
            }
            else if (heading == EAST) {
                if (y != 0){
                    setH(x, y - 1);
                }
                 
            }
             
        }
        //FRONTWALL
        if (frontWall)
        {
            if (heading == NORTH)
            {
                setH(x, y);
            }
            else if (heading == SOUTH)
            {
                if (y != 0)
                {
                    setH(x, y - 1);
                }
            }
            else if (heading == WEST)
            {
                if (x != 0)
                {
                    setV(x - 1, y);
                }
                 
            }
            else if (heading == EAST)
            {
                setV(x, y);
            }
        }
         

         
        switch (heading)
        {
            case NORTH:
                frontX = x;
                frontY = y + 1;
                rightX = x + 1;
                rightY = y;
                leftX = x - 1;
                leftY = y;
                break;
            case SOUTH:
                frontX = x;
                frontY = y - 1;
                rightX = x - 1;
                rightY = y;
                leftX = x + 1;
                leftY = y;
                break;
            case EAST:
                frontX = x + 1;
                frontY = y;
                rightX = x;
                rightY = y - 1;
                leftX = x;
                leftY = y + 1;
                break;
            case WEST:
                frontX = x - 1;
                frontY = y;
                rightX = x;
                rightY = y + 1;
                leftX = x;
                leftY = y - 1;
                break;
            default:
                break;
        }
         
         
        //if (!frontWall) {
        if (!frontWall && c_array[x][y].m_distance > c_array[frontX][frontY].m_distance)
        {
            return MoveForward;
        }
         
        if (!rightWall && c_array[x][y].m_distance > c_array[rightX][rightY].m_distance)
        {
            heading = clockwise(heading);
            return TurnClockwise;
        }
         
        if (!leftWall && c_array[x][y].m_distance > c_array[leftX][leftY].m_distance)
        {
            heading = counterClockwise(heading);
            return TurnCounterClockwise;
        }
         
        if (frontWall && rightWall && leftWall)
        {
            heading = opposite(heading);
            return TurnAround;
        }
         

         
        //need to edit more for corner cases
        if (c_array[x][y].m_distance != 0)
        {
					
				if ((heading == NORTH && c_array[x][y].m_distance > c_array[x][y-1].m_distance && !isWallBetween(c_array[x][y], c_array[x][y-1])) || (heading == SOUTH && c_array[x][y].m_distance > c_array[x][y+1].m_distance && !isWallBetween(c_array[x][y], c_array[x][y+1])) || (heading == EAST && c_array[x][y].m_distance > c_array[x-1][y].m_distance && !isWallBetween(c_array[x][y], c_array[x-1][y])) || (heading == WEST && c_array[x][y].m_distance > c_array[x+1][y].m_distance && !isWallBetween(c_array[x][y], c_array[x+1][y])))
        {
            heading = opposite(heading);
            return TurnAround;
        }
				
            //std::cout << "Running Floodfill" << std::endl;
            floodFill(x, y);
            return nextMovement();
        }
        else
        {
            shouldGoToCenter = !shouldGoToCenter;
            if (shouldGoToCenter)
            {
                //std::cout << "Went back to the beginning. Now to go back to the center" << std::endl;
                calculateManahattan(7, 7);
                 
                //Call Floodfill on every cell
                for (i = 0; i < 16; i++)
                {
                    for (j = 0; j < 16; j++)
                    {
                        floodFill(i, j);
                    }
                }
                heading = opposite(heading);
                return TurnAround;
            }
            else
            {
                //std::cout << "Found center! Now to go back to the beginning" << std::endl;
                calculateManahattan(0, 0);
                floodFill(x, y);
                heading = opposite(heading);
                return TurnAround;
            }
        }
    }




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
	if(direction == 1)
	{
		while (angle-curAng < degrees) {
			printf("angle: %d\tcurAngle: %d\tangle-curAngle: %d\r\n", angle, curAng, angle-curAng); 
			targetLeft = -50*direction;
			targetRight = 50*direction;
		}
	}
	else if(direction == -1)
	{
			while (angle-curAng > degrees) {
			printf("angle: %d\tcurAngle: %d\tangle-curAngle: %d\r\n", angle, curAng, angle-curAng); 
			targetLeft = -50*direction;
			targetRight = 50*direction;
		}
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

	if (mouseStarted == 1) {
		readGyro();
		readVolMeter();
		readSensor();
		PID();
		left_enc = getLeftEncCount();
		right_enc = getRightEncCount();  
	}
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
	int i;
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
	
	
	displayMatrix("CALB");
	for (i = 0; i < 1000; i++) {
		readSensor();
		leftMiddleValue += DLSensor;
		rightMiddleValue += DRSensor;
	}
	leftMiddleValue /= 1000;
	rightMiddleValue /= 1000;
	hasLeftWall = leftMiddleValue * 0.75;
	hasRightWall = rightMiddleValue * 0.75;

	if (1) {
		mouseStarted = 1;
		forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);
		forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);
		forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);
		forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);
		forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, false);
		stop(1000);
		turnRightAngleLeft(runSpeed, runSpeed);
		stop(1000);
		forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, false);
		stop(1000);
		turnRightAngleLeft(runSpeed, runSpeed);
		stop(1000);
		forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);
		forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, false);
		stop(1000);
		turnRightAngleRight(runSpeed, runSpeed);
		stop(1000);
		forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, false);
		stop(1000);
		turnRightAngleRight(runSpeed, runSpeed);
		stop(1000);
		forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, false);
		stop(1000);
		turnRightAngleLeft(runSpeed, runSpeed);
		stop(1000);
		forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, false);
		stop(1000);
		turnRightAngleRight(runSpeed, runSpeed);
		stop(1000);
		forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, false);
		stop(1000);
		turnRightAngleLeft(runSpeed, runSpeed);
		stop(1000);
		forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, false);
		stop(1000);
		turnDegrees(-15000, -1);//turn right
		stop(10000);
	}
	else {
		while (1) {		
			readSensor();
			if((DLSensor > hasLeftWall && DRSensor > hasRightWall))//has both walls
			{
				displayMatrix("BOTH");
			}        
			else if((DLSensor > hasLeftWall))//only has left wall
			{
				displayMatrix("LWAL");
			}
			else if((DRSensor > hasRightWall))//only has right wall
			{
				displayMatrix("RWAL");
			}
			else if((DLSensor < hasLeftWall && DRSensor <hasRightWall))//no wall, use encoder or gyro
			{
				displayMatrix("NONE");
			}		
			delay_ms(500);
		}
	}

}

