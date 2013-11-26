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
}


void setMotor(int SpeedA, int SpeedB){
  motor1.write(SpeedA);
  motor2.write(SpeedB);
}

int i = 0;
void loop(){
  i += 1;
  if (i%50 == 0){
    setMotor(i,i);
  }
  delay(250);
}
