#define motorA 10
#define motorB 6
#define MINMOTORVAL 30
#define MIDMOTORVAL 90
#define MAXMOTORVAL 150
#define MINMOTORINPUT -100
#define MAXMOTORINPUT 100
#define MINPPMVAL 1400
#define MAXPPMVAL 1600
// #include <Servo.h>

int motor_flag; 
int motor_1; 
int motor_2;
int PPMin = 4;
int PPMinAil = 3;
int PPMinElev = 2;

int valueL=90;
int valueR=90;

void setup()
{
  Serial.begin(9600); //Iniciamos com serial/ 
  pinMode(PPMin, INPUT); //Patita 4 como entrada / Pin 4 as input
  pinMode(PPMinAil, INPUT); //Patita 4 como entrada / Pin 4 as input
  pinMode(PPMinElev, INPUT); //Patita 4 como entrada / Pin 4 as input
} 

void loop()
{
  int m_speed = 0;
  int m_dir = 0;
  //waits ultil synchronize arrives > 4 miliseconds
  if(pulseIn(PPMin , HIGH) > 4000); //If pulse > 4 miliseconds, continues
  { 
    motor_flag = pulseIn(PPMin, HIGH);
    if(motor_flag > 1500){ // MOTORS OFF
      Serial.println("OFF");
    } else { Serial.println("ON"); } // MOTORS ON
  }  


  if(pulseIn(PPMinAil , HIGH) > 4000); //If pulse > 4 miliseconds, continues
  { 
    m_speed = map(pulseIn(PPMinAil, HIGH), MINPPMVAL, MAXPPMVAL, -100, 100);
  }  


  if(pulseIn(PPMinElev , HIGH) > 4000); //If pulse > 4 miliseconds, continues
  { 
    m_dir = map(pulseIn(PPMinElev, HIGH), MINPPMVAL, MAXPPMVAL, -100, 100);
  }
  motor_1 = m_speed + m_dir;
  motor_2 = m_speed - m_dir;
  if(motor_1 > MAXMOTORINPUT) { motor_1 = MAXMOTORINPUT; }
  else if (motor_1 < MINMOTORINPUT) { motor_1 = MINMOTORINPUT; }
  if(motor_2 > MAXMOTORINPUT) { motor_2 = MAXMOTORINPUT; }
  else if (motor_2 < MINMOTORINPUT) { motor_2 = MINMOTORINPUT; }

  Serial.print("M1: ");
  Serial.print(motor_1);
  Serial.print(", M2: ");
  Serial.println(motor_2);

  delay(250);
}  