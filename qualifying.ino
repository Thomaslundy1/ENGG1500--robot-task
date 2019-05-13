#include <Servo.h>

//Distances from the centroid of the robot to the centre of each sensor in mm 
float x1 = -22.5;
float x2 = -7.5;
float x3 = 7.5;
float x4 = 22.5;

//Variables to store each data point in
float w1;
float w2;
float w3;
float w4;

//Variables for storing the numerator and denominator of Equation 1
float den = 0;   
float num = 0;

// These #define statements make the code more readable.
// Instead of a pin number "7" or "12" we can write "ECHO" or "TRIG"
#define ECHO A4
#define TRIG A0

Servo servo;  
int angle = 0;   // servo position in degrees 

void setup() {
  pinMode(ECHO,INPUT); //Initialise pin 12 as an input
  pinMode(TRIG,OUTPUT); //Initialise pin 7 as an output
  Serial.begin(9600); //begin serial communication   
  pinMode(A1, INPUT); //Far right line sensor
  pinMode(A2, INPUT); //Inner right line sensor
  pinMode(A3, INPUT); //Inner left line sensor
  pinMode(A5, INPUT); //Far left line sensor
  pinMode(5,OUTPUT); //set ENA as an output
  pinMode(6,OUTPUT); //set ENB as an output
  pinMode(8,OUTPUT); //set IN1 as an output
  pinMode(9,OUTPUT); //set IN2 as an output
  pinMode(10,OUTPUT); //set IN3 as an output
  pinMode(11,OUTPUT); //set IN4 as an output
  servo.attach(13); 
   
}
  
void loop() {
  unsigned int distance_mm = 0;
  w1 = analogRead(A1);
  w2 = analogRead(A2);
  w3 = analogRead(A3);
  w4 = analogRead(A5);

  num = (w1 * x1) + (w2 * x2) + (w3 * x3) + (w4 * x4);
  den = w1 + w2 + w3 + w4;
  distance_mm = sonar_mm();
  float lineDist = num/den;  
  float error = 0 - lineDist;
  float Kp = 3.2;
  int leftPwm = 62 + Kp*error; 
  int rightPwm = 62 - Kp*error;
  leftForwards();
  rightForwards();
  Serial.print("Distance from line = ");
  Serial.println(lineDist);
  analogWrite(5, leftPwm);
  analogWrite(6, rightPwm);
  if(distance_mm <= 100){
      analogWrite(5,0);  
      analogWrite(6,0);
  }
}
    
void leftForwards(void) //This function sets IN1 = LOW and IN2 = HIGH in order to set the direction to forwards for motor 1
{
  digitalWrite(8,HIGH); //IN1
  digitalWrite(9,LOW); //IN2
}

void leftBackwards(void) //This function sets IN1 = HIGH and IN2 = LOW in order to set the direction to backwards for motor 1
{
  digitalWrite(8,LOW); //IN1
  digitalWrite(9,HIGH); //IN2
}

void rightForwards(void)  //This function sets IN3 = LOW and IN4 = HIGH in order to set the direction to forwards for motor 2
{
  digitalWrite(10,HIGH); //IN3
  digitalWrite(11,LOW); //IN4
}
void rightBackwards(void) //This function sets IN3 = HIGH and IN4 = LOW in order to set the direction to forwards for motor 2
{
  digitalWrite(10,LOW); //IN3
  digitalWrite(11,HIGH); //IN4
}

unsigned int sonar_mm(void)
{
  long duration = 0;
  const float speed_sound = 340.29;// m/s, "const" makes the compiler able to optimise the program where this variable is used, cool!
  // Read in a distance from the ultrasonic distance sensor:
  // The ultrasonic burst is triggered by a HIGH pulse of 10 microseconds.
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  //read length of time pulse
  duration = pulseIn(ECHO, HIGH); //This function measures a pulsewidth and returns the width in microseconds
  // convert the time into a distance
  // the code "(unsigned int)" turns the result of the distance calculation
  // into an integer instead of a floating point (decimal or fractional) number.
  return (unsigned int)(0.5 * duration * 1e-6 * speed_sound * 1e3); 
  //"unsigned" ensures we are returning an unsigned number, remember that there is no such thing as negative distance.
}
