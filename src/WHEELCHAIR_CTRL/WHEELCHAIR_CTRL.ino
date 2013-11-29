#define motorA 10
#define motorB 6
#define MINMOTORVAL 30
#define MIDMOTORVAL 90
#define MAXMOTORVAL 150
#define MINMOTORINPUT 0
#define MAXMOTORINPUT 255
#define MINPPMVAL 1400
#define MAXPPMVAL 1600
#include <Servo.h>

int motor_flag; 
int autonomous = 0;
int PPMin = 4;
int PPMinAil = 3;
int PPMinElev = 2;
int PPMinAutonomous = 5;
int StatusLEDpin = 13;

Servo motor1;
Servo motor2;
String motorcommand;

int valueL=90;
int valueR=90;

void setup(){
  motor1.attach(motorA);
  motor2.attach(motorB);
  Serial.begin(9600);
  pinMode(PPMin, INPUT); //Patita 4 como entrada / Pin 4 as input
  pinMode(PPMinAil, INPUT); //Patita 4 como entrada / Pin 4 as input
  pinMode(PPMinElev, INPUT); //Patita 4 como entrada / Pin 4 as input
  pinMode(StatusLEDpin, OUTPUT);
}


void setMotor(int SpeedA, int SpeedB){
  if(motor1.attached()){
    motor1.write(SpeedA);
    // Serial.print("MOTOR1 Set");
  }
  if(motor2.attached()){
    motor2.write(SpeedB);
    // Serial.print("MOTOR2 Set");
  }
}

void check_E_stop(){
  //waits ultil synchronize arrives > 4 miliseconds
  if(pulseIn(PPMin , HIGH) > 4000); //If pulse > 4 miliseconds, continues
  { 
    motor_flag = pulseIn(PPMin, HIGH);
    if(motor_flag > 1500){ // MOTORS OFF
      Serial.println("OFF");
      if(motor1.attached()){
        motor1.detach();
      }
      if(motor2.attached()){
        motor2.detach();
      }
    } else {
      Serial.println("ON");
      if(!motor1.attached()){
        motor1.attach(motorA);
      }
      if(!motor2.attached()){
        motor2.attach(motorB);
      }
    } // MOTORS ON
  }
}

void check_autonomous(){
  //waits ultil synchronize arrives > 4 miliseconds
  if(pulseIn(PPMinAutonomous , HIGH) > 4000); //If pulse > 4 miliseconds, continues
  { 
    if(pulseIn(PPMinAutonomous, HIGH) > 1500){ autonomous = 1; }
    else { autonomous = 0; }
  }
}

byte index = 0;

int* motorFromSerial(){
  
}

int StatusLED = 0;

void loop(){
  check_E_stop();
  check_autonomous();
  int* val;
  if(autonomous){
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
      } else if(motorcommand[0] == 'R') {
        // valueR = map(atoi(value), MINMOTORINPUT, MAXMOTORINPUT, MINMOTORVAL, MAXMOTORVAL);
        valueR = atoi(value);
      }

      valueL = map(valueL, MINMOTORINPUT, MAXMOTORINPUT, MINMOTORVAL, MAXMOTORVAL);
      valueR = map(valueR, MINMOTORINPUT, MAXMOTORINPUT, MINMOTORVAL, MAXMOTORVAL);
    }

    digitalWrite(StatusLEDpin, StatusLED);
    StatusLED = !StatusLED;
    Serial.println("AUTO");


  } else {
    int m_speed = 0;
    int m_dir = 0;

    if(pulseIn(PPMinAil , HIGH) > 4000); //If pulse > 4 miliseconds, continues
    { 
      m_speed = map(pulseIn(PPMinAil, HIGH), MINPPMVAL, MAXPPMVAL, MINMOTORINPUT, MAXMOTORINPUT);
    }  

    if(pulseIn(PPMinElev , HIGH) > 4000); //If pulse > 4 miliseconds, continues
    { 
      m_dir = map(pulseIn(PPMinElev, HIGH), MINPPMVAL, MAXPPMVAL, MINMOTORINPUT, MAXMOTORINPUT);
    }
    valueL = m_speed + m_dir;
    valueR = m_speed - m_dir;
    if(valueL > MAXMOTORINPUT) { valueL = MAXMOTORINPUT; }
    else if (valueL < MINMOTORINPUT) { valueL = MINMOTORINPUT; }
    if(valueR > MAXMOTORINPUT) { valueR = MAXMOTORINPUT; }
    else if (valueR < MINMOTORINPUT) { valueR = MINMOTORINPUT; }


    digitalWrite(StatusLEDpin, HIGH);
    Serial.println("MAN");
    valueL = map(valueL, MINMOTORINPUT, MAXMOTORINPUT, MINMOTORVAL, MAXMOTORVAL);
    valueR = map(valueR, MINMOTORINPUT, MAXMOTORINPUT, MINMOTORVAL, MAXMOTORVAL);
  }

  if (index > 0) { // If we received a command this loop
    Serial.print("\nL");
    Serial.print(valueL);
    Serial.print(" R");
    Serial.print(valueR);
    Serial.print(" PULSEIN");
    Serial.println(motor_flag);
  }
  Serial.flush();
  setMotor(valueL, valueR);

  delay(250);

}
