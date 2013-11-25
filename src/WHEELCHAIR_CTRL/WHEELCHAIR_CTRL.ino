#define motorA 10
#define motorB 6
#include <Servo.h>

Servo motor1;
Servo motor2;
String motorcommand;

int valueL;
int valueR;

void setup(){
  motor1.attach(motorA);
  motor2.attach(motorB);
  Serial.begin(9600);
}


void setMotor(int SpeedA, int SpeedB){
//  int newSpeedA = map(speedA, 0, 100, 30, 150);
  motor1.write(SpeedA);
//  int newSpeedB = map(speedB, 0, 100, 30, 150);
  motor2.write(SpeedB);
}

void loop(){
  byte index = 0;
  
  char inChar = 0;
  char inData[20];
  char motorcommand[8];
  //Fill buffer up:
  index = 0;
  while (Serial.available() > 0) {
    //Buffer
    // read the incoming byte:
    if(index<19){
      inChar = Serial.read();
      Serial.write(inChar);
      Serial.flush();
      inData[index] = inChar;
      index++;
      inData[index] = '\0';
    }         
  }
    //Get value from array
  motorcommand[0] = '\0';
  for (int i=0;i<19;i++){
    char sensorValue = inData[i]; 
    if (sensorValue == '\0'){
      break;
    }
    motorcommand += sensorValue;    
  }
  //Print out what you got:
  //Serial.println(motorcommand);
  char value[3];
  value[0] = motorcommand[1];
  value[1] = motorcommand[2];
  value[2] = motorcommand[3];
  if(motorcommand[0] == 'L'){
    valueL = atoi(value);
  } else if(motorcommand[0] == 'R') {
    valueR = atoi(value);
  }
  setMotor(valueL,valueR);
}
