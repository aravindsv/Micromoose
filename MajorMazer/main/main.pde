#include "pinDefinitions.h"

unsigned long countLeft = 0;
unsigned long countRight = 0;

//declare functions
void initPinModes();
void interruptHandler();
void motorTest();
void encoderTest();
void gyroTest();
void sensorTest();
void buttonTest();

void setup(){
  
  pinMode(BOARD_LED_PIN, OUTPUT);
  initPinModes();

}

void loop(){
  
  //attach interrupts to encoder outputs
  attachInterrupt(encoderAOutA, interruptAOUTA, CHANGE);
  attachInterrupt(encoderAOutB, interruptAOUTB, CHANGE);  
  attachInterrupt(encoderBOutA, interruptBOUTA, CHANGE);
  attachInterrupt(encoderBOutB, interruptBOUTB, CHANGE);
  
  //start motor
  digitalWrite(motorStandby, 1);
  //spin forwards
  digitalWrite(motorALogic1, 1); digitalWrite(motorALogic2, 0);
  digitalWrite(motorBLogic1, 1); digitalWrite(motorBLogic2, 0);
  
  //turn on motors at low speed
  analogWrite(motorAPWM, 10000); analogWrite(motorBPWM, 12600);

  
}

void interruptAOUTA() {
    countRight++;
}

void interruptAOUTB() {
    countRight++;
}

void interruptBOUTA() {
    countLeft++;
}


void interruptBOUTB() {
    countLeft++;
}

//encoder test: print count from encoder
void encoderTest(){
  //blink onboard LED on enocder count
  //attach interrupt to encoder output
  attachInterrupt(encoderAOutA, interruptAOUTA, CHANGE);
  attachInterrupt(encoderAOutB, interruptAOUTB, CHANGE);  
  
  //enable motor
  digitalWrite(motorStandby, 1);
  
  //turn on motors
  digitalWrite(motorALogic1, 1); digitalWrite(motorALogic2, 0);
  digitalWrite(motorBLogic1, 1); digitalWrite(motorBLogic2, 1);
  
  //quick ramp up to max speed
  for(int i = 0; i < 65535; i++){
    analogWrite(motorAPWM, i); delayMicroseconds(25);
  }
  
  delay(500);
  
  //quick slow down to stop
  for(int i = 65535; i > -1; i--){
    analogWrite(motorAPWM, i); delayMicroseconds(25);
  }
  
  delay(500);
  
  SerialUSB.println(countLeft);
  countLeft = 0;
}

//motor test: spin motors in opposite directions, speed ramp up and slow down
void motorTest(){
  //enable motor
  digitalWrite(motorStandby, 1);
  
  //set opposite directions
  digitalWrite(motorALogic1, 1); digitalWrite(motorALogic2, 0);
  digitalWrite(motorBLogic1, 0); digitalWrite(motorBLogic2, 1);
  
  //quick ramp up to max speed
  for(int i = 0; i < 65535; i++){
    analogWrite(motorAPWM, i); analogWrite(motorBPWM, i); delayMicroseconds(10);
  }
  
  delay(1000);
  
  //quick slow down to stop
  for(int i = 65535; i > -1; i--){
    analogWrite(motorAPWM, i); analogWrite(motorBPWM, i); delayMicroseconds(10);
  }
  
  delay(500);
  
  //reverse direcitons of both motors
  digitalWrite(motorALogic1, 0); digitalWrite(motorALogic2, 1);
  digitalWrite(motorBLogic1, 1); digitalWrite(motorBLogic2, 0);
  
  //quick ramp up to max speed
  for(int i = 0; i < 65535; i++){
    analogWrite(motorAPWM, i); analogWrite(motorBPWM, i); delayMicroseconds(10);
  }
  
  delay(1000);
  
  //quick slow down to stop
  for(int i = 65535; i > -1; i--){
    analogWrite(motorAPWM, i); analogWrite(motorBPWM, i); delayMicroseconds(10);
  }
  
  delay(500);
}

void gyroTest(){
  //make sure motor is disabled
  digitalWrite(motorStandby, 0);
  
  //read analog values from gyro
  int z1x = analogRead(gyroZ1X);
  int z4x = analogRead(gyroZ4X);
  int vref = analogRead(gyroVref);
  
  //print values to serial monitor
  SerialUSB.println(z1x);
  SerialUSB.println(z4x);
  SerialUSB.println(vref);
  SerialUSB.println("-------");
  
  //do the above every 50ms
  delay(50);  
}

void sensorTest(){
  //make sure motor is disabled
  digitalWrite(motorStandby, 0);
  
  //read from sensor1
  digitalWrite(emitter1, 1); delayMicroseconds(10);
  int s1 = analogRead(sensor1);
  digitalWrite(emitter1, 0); delayMicroseconds(10);
  
  //read from sensor2
  digitalWrite(emitter2, 1); delayMicroseconds(10);
  int s2 = analogRead(sensor2);
  digitalWrite(emitter2, 0); delayMicroseconds(10);
  
  //read from sensor3
  digitalWrite(emitter3, 1); delayMicroseconds(10);
  int s3 = analogRead(sensor3);
  digitalWrite(emitter3, 0); delayMicroseconds(10);
  
  //read from sensor4
  digitalWrite(emitter4, 1); delayMicroseconds(10);
  int s4 = analogRead(sensor4);
  digitalWrite(emitter4, 0); delayMicroseconds(10);
  
  //print sensor values to serial monitor
  SerialUSB.println(s1);
  SerialUSB.println(s2);
  SerialUSB.println(s3);
  SerialUSB.println(s4);
  SerialUSB.println("-------");
  
  //do the above every 50ms
  delay(50);
  
}

void buttonTest(){
  //make sure motor is disabled
  digitalWrite(motorStandby, 0);
  
  //if any of the buttons are high, toggle LED and debounce for 200ms
  if(digitalRead(button1)){
    toggleLED(); delay(200);
  }
  
  if(digitalRead(button2)){
    toggleLED(); delay(200);
  }
  
  if(digitalRead(button3)){
    toggleLED(); delay(200);
  }
}

//init pinmodes for all pins used on maple
void initPinModes(){
    //set pinmode for emitter pins
  pinMode(emitter1, OUTPUT); pinMode(emitter2, OUTPUT); pinMode(emitter3, OUTPUT); pinMode(emitter4, OUTPUT);
  
  //set pinmode for sensor pins
  pinMode(sensor1, INPUT_ANALOG); pinMode(sensor2, INPUT_ANALOG); pinMode(sensor2, INPUT_ANALOG); pinMode(sensor2, INPUT_ANALOG); 
  
  //set pinmode for motor pins
  pinMode(motorStandby, OUTPUT);
  pinMode(motorALogic1, OUTPUT); pinMode(motorALogic2, OUTPUT); pinMode(motorAPWM, PWM);
  pinMode(motorBLogic1, OUTPUT); pinMode(motorBLogic2, OUTPUT); pinMode(motorBPWM, PWM);
  
  //set pinmode for encoder pins
  pinMode(encoderAOutA, INPUT); pinMode(encoderAOutB, INPUT);
  pinMode(encoderBOutA, INPUT); pinMode(encoderBOutB, INPUT);
  
  //set pinmode for gyro pins
  pinMode(gyroHP, OUTPUT); pinMode(gyroPD, OUTPUT);
  pinMode(gyroZ1X, INPUT_ANALOG); pinMode(gyroZ4X, INPUT_ANALOG);
  pinMode(gyroVref, INPUT_ANALOG);
 
  //set pinmode for other periferal pins 
  pinMode(button1, INPUT); pinMode(button2, INPUT); pinMode(button3, INPUT);
  pinMode(batteryVoltage, INPUT_ANALOG);
  
}
