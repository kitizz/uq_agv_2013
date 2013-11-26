#define motorA 10
#define motorB 6
#include <Servo.h>

Servo motor1;
Servo motor2;
String motorcommand;

int valueL=90;


int valueR=90;

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
  char inData[5];
  char motorcommand[8];
  //Fill buffer up:
  index = 0;
  if (Serial.available() > 0) {
    while (index<4) {
      //Buffer
      // read the incoming byte:
      if (Serial.available() == 0) continue;
      inChar = Serial.read();
      
      Serial.write(inChar);
      Serial.flush();
      
      if (inChar == '\n') break;
      
      inData[index] = inChar;
      index++;
      inData[index] = '\0';
    }
    Serial.println("...received");
  }
    //Get value from array
  motorcommand[0] = '\0';
  for (int i=0;i<4;i++){
    char sensorValue = inData[i]; 
    if (sensorValue == '\0'){
      break;
    }
    motorcommand[i] = sensorValue;
    motorcommand[i+1] = '\0';    
  }
  //Print out what you got:
  //Serial.println(motorcommand);
  char value[4];
  value[0] = motorcommand[1];
  value[1] = motorcommand[2];
  value[2] = motorcommand[3];
  value[3] = '\0';
  if(motorcommand[0] == 'L'){
    valueL = atoi(value);
  } else if(motorcommand[0] == 'R') {
    valueR = atoi(value);
  }
  if (index > 0) { // If we received a command this loop
    Serial.print("\nL");
    Serial.print(valueL);
    Serial.print(" R");
    Serial.println(valueR);
  }
  Serial.flush();
  setMotor(valueL,valueR);
}
