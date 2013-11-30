#define motorA 10
#define motorB 6
#define switch 2
#include <Servo.h>

Servo motor1;
Servo motor2;
String motorcommand;


int valueL;
int valueR;

int motorsON = 1;

void setup(){
  motor1.attach(motorA);
  motor2.attach(motorB);

  pinMode(switch, INPUT);
}


void setMotor(int SpeedA, int SpeedB){
  motor1.write(SpeedA);
  // motor2.write(SpeedB);
}

int i = 0;
void loop(){
  if(motorsON){
    i += 1;
    if (i%50 == 0){
      setMotor(i,i);
    }    
  }

  if(digitalRead(2)){
    delay(50);
    if(digitalRead(2)){
        motorsON != motorsON;

        if(motorsON){
          motor1.attach(motorA);
          motor2.attach(motorB);
        } else {
          motor1.detach();
          motor2.detach();
        }
    }
  }

  delay(250);
}
