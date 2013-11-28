// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 
#define OFF 45
#define ON 120
#define joyX A0
#define joyY A1

#define motorA 10
#define motorB 6
//#define debug

#include <Servo.h> 
byte index = 0;
char inChar = 0;
char inData[5];
String stringOne = "Sensor value: ";

//Servo myservo;  // create servo obj



//ect to control a servo 
Servo motor1;
Servo motor2;

//int potpin = 0;  // analog pin used to connect the potentiometer
int val1;    // variable to read the value from the analog pin 
int val2;

String serial(){
  //Fill buffer up:
  index = 0;
  while (Serial.available() > 0) {
    //Buffer
    // read the incoming byte:
    if(index<4){
      inChar = Serial.read();
      Serial.write(inChar);
      Serial.flush();
      inData[index] = inChar;
      index++;
      inData[index] = '\0';
    }         
  }
  //Get value from array
  String motorcommand = "";
  for (int i=0;i<4;i++){
    char sensorValue = inData[i]; 
    if (sensorValue == '\0'){
      break;
    }
    motorcommand += sensorValue;    
  }
  //Print out what you got:
#ifdef debug
  Serial.println(stringOne + motorcommand);
#endif

  //Clear serial & buffer
  Serial.flush();
  return motorcommand;

}

void motorSend(String motorCommand){  
  //Detect what motor
  int motorspeed = 0;

  if(motorCommand[0] == 'L'){
    char derp[3];
    derp[0] = motorCommand[1];
    derp[1] = motorCommand[2];
    derp[2] = motorCommand[3];
#ifdef debug
    Serial.println(atoi(derp));
#endif
    //Send to sabertooth
    motor1.write(atoi(derp));
    //    Serial.write(atoi(derp));
  }
  else if(motorCommand[0] == 'R'){
    char derp[3];
    derp[0] = motorCommand[1];
    derp[1] = motorCommand[2];
    derp[2] = motorCommand[3];
#ifdef debug
    Serial.println(atoi(derp));
#endif
    //Send to sabertooth
    motor2.write(atoi(derp));
    //    Serial.write(atoi(derp));    
  }
  else {
    //#ifdef debug
    Serial.println("NO COMMAND DETECTED");
    //#endif
  };
  motorCommand[0];
}

void setup() 
{ 
  pinMode(joyX, INPUT);
  pinMode(joyY, INPUT);

  Serial.begin(9600);
  motor1.attach(motorA);
  motor2.attach(motorB);
//  myservo.attach(7);  // attaches the servo on pin 9 to the servo object 
} 

void loop() 
{ 
  //GET SERIAL SHIZ
  String motorcommand = serial();
  //SEND TO SABERTOOTH
  motorSend(motorcommand);
  //  val1 = analogRead(joyX);
  //  val1 = map(val1, 0, 1023, 0, 179);
  //  val2 = analogRead(joyY);
  //  val2 = map(val2, 0, 1023, 0, 179);  

  //  myservo.write(ON);                  // sets the servo position according to the scaled value 
  //  delay(2000);                           // waits for the servo to get there 
  //  myservo.write(OFF);                  // sets the servo position according to the scaled value 
  //  delay(2000);                           // waits for the servo to get there 
} 


