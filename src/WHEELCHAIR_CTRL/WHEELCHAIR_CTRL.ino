#define motorA 10
#define motorB 6
#define MINMOTORVAL 930
#define MAXMOTORVAL 1930
#define MINMOTORINPUT 0
#define MAXMOTORINPUT 100
#define MINPPMVAL 1350
#define MAXPPMVAL 1650
// #include <Servo.h>

int motor_flag; 
int autonomous = 0;
int PPMin = 4;
int PPMinAil = 3;
int PPMinElev = 2;
int PPMinAutonomous = 5;
int StatusLEDpin = 13;

String motorcommand;

int valueL=90;
int valueR=90;

void setup(){
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  Serial.begin(115200);
  // Serial.setTimeout(1000);
  pinMode(PPMin, INPUT); //Patita 4 como entrada / Pin 4 as input
  pinMode(PPMinAil, INPUT); //Patita 4 como entrada / Pin 4 as input
  pinMode(PPMinElev, INPUT); //Patita 4 como entrada / Pin 4 as input
  pinMode(StatusLEDpin, OUTPUT);

  setMotor(50,50);
}


void setMotorIndividual(int motor, int Speed){
  digitalWrite(motor, HIGH);
  delayMicroseconds(map(Speed, 0, 100, MINMOTORVAL, MAXMOTORVAL));
  digitalWrite(motor, LOW);
}

void setMotor(int SpeedA, int SpeedB){
  setMotorIndividual(motorA, SpeedA);
  setMotorIndividual(motorB, SpeedB);
}

bool check_E_stop(){
  //waits ultil synchronize arrives > 4 miliseconds
  if(pulseIn(PPMin , HIGH) > 4000); //If pulse > 4 miliseconds, continues
  { 
    motor_flag = pulseIn(PPMin, HIGH);
    if(motor_flag > 1500){ // MOTORS OFF
      Serial.println("OFF");     
      return false;
    } else {
      Serial.println("ON");
      return true;
    } // MOTORS ON
  }
}

int prevMode = 0;

void check_autonomous(){
  //waits ultil synchronize arrives > 4 miliseconds
  if(pulseIn(PPMinAutonomous , HIGH) > 4000); //If pulse > 4 miliseconds, continues
  {
    if(pulseIn(PPMinAutonomous, HIGH) > 1500){ autonomous = 1; }
    else { autonomous = 0; }
  }
  if(prevMode != autonomous){
    setMotor(50,50);
    valueL = 50;
    valueR = 50;
    prevMode = autonomous;
  }
}

byte index = 0;


int StatusLED = 0;

void loop(){
  check_autonomous();
  // valueL = 50;
  // valueR = 50;
  //autonomous = 1;
  //autonomous = 1;
  if(autonomous){
    char inChar = 0;
    char inData[5];
    char motorcommand[8];
    //Fill buffer up:
    index = 0;
    if (Serial.available() > 0) {
      unsigned long prevTime = millis();
      long interval = 500; // Interval for serial check in ms
      while (index<4) {
        unsigned long curTime = millis();
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
        // The reading of the serial is taking too long (error), ignore and break
        if((curTime - prevTime) > interval){ break; }
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
    valueL = 50 + ((m_speed - 50) + (m_dir-50));
    valueR = 50 + ((m_speed - 50) - (m_dir-50));
    // valueL = m_speed;
    // valueR = m_dir;
    if(valueL > MAXMOTORINPUT) { valueL = MAXMOTORINPUT; }
    else if (valueL < MINMOTORINPUT) { valueL = MINMOTORINPUT; }
    if(valueR > MAXMOTORINPUT) { valueR = MAXMOTORINPUT; }
    else if (valueR < MINMOTORINPUT) { valueR = MINMOTORINPUT; }


    digitalWrite(StatusLEDpin, HIGH);
    Serial.println("MAN");
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
  if(check_E_stop()){
    setMotor(valueL, valueR);  
  } else {
    setMotor(50,50);
  }

  delay(150);

}
