#define motorA 10
#define motorB 6
#define switch 2
#include <Servo.h>

// Servo motor1;
// Servo motor2;
String motorcommand;


int valueL;
int valueR;

int motorsON = 1;

void setup(){
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  pinMode(switch, INPUT);
  Serial.begin(9600);
}


void setMotorIndividual(int motor, int Speed){
  digitalWrite(motor, HIGH);
  delayMicroseconds(map(Speed, 0, 100, 930, 1930));
  digitalWrite(motor, LOW);
}

void setMotor(int SpeedA, int SpeedB){
  setMotorIndividual(motorA, SpeedA);
  setMotorIndividual(motorB, SpeedB);
}

int i = 0;
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
      
      // ECHO BACK TO PC
      // Serial.write(inChar);
      // Serial.flush();
      
      if (inChar == '\n') break;
      
      inData[index] = inChar;
      index++;
      inData[index] = '\0';
    }
    Serial.println("...received");

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
      // valueL = map(atoi(value), MINMOTORINPUT, MAXMOTORINPUT, MINMOTORVAL, MAXMOTORVAL);
      valueL = atoi(value);
      Serial.print(valueL);
    } else if(motorcommand[0] == 'R') {
      // valueR = map(atoi(value), MINMOTORINPUT, MAXMOTORINPUT, MINMOTORVAL, MAXMOTORVAL);
      valueR = atoi(value);
      Serial.print(valueR);
    }

    // valueL = map(valueL, MINMOTORINPUT, MAXMOTORINPUT, MINMOTORVAL, MAXMOTORVAL);
    // valueR = map(valueR, MINMOTORINPUT, MAXMOTORINPUT, MINMOTORVAL, MAXMOTORVAL);
  }
  setMotor(valueL, valueR);

  delay(250);
}
