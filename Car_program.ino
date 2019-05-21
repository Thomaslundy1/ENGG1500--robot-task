#include <Servo.h>


// These #define statements make the code more readable.
// Instead of a pin number "7" or "12" we can write "ECHO" or "TRIG"
#define ECHO A4
#define TRIG A0
#define ECHO1 7
#define TRIG1 4

int servoPin = 13;
  Servo servo;  
  int angle = 0;   // servo position in degrees 

int servoPin2 = 12;
  Servo servo2;  
  int angle2 = 0;   // servo position in degrees 

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

void setup() {
  pinMode(ECHO,INPUT); //Initialise pin 12 as an input
  pinMode(TRIG,OUTPUT); //Initialise pin 7 as an output
  pinMode(ECHO1,INPUT);
  pinMode(TRIG1,OUTPUT);
  Serial.begin(9600); //begin serial communication 
  pinMode(5,OUTPUT); //set ENA as an output
  pinMode(6,OUTPUT); //set ENB as an output
  pinMode(8,OUTPUT); //set IN1 as an output
  pinMode(9,OUTPUT); //set IN2 as an output
  pinMode(10,OUTPUT); //set IN3 as an output
  pinMode(11,OUTPUT); //set IN4 as an output
  servo.attach(13);
  servo2.attach(12); 
   
}
  
void loop() {

unsigned long time1 = millis();
  if (time1 < 1000){
    analogWrite(5,100);
    analogWrite(6,100);
  }
 
    for(angle = 20; angle < 160; angle=angle+10)  
  { 
      unsigned int distance_mm = 0; //This variable will hold the distance
      unsigned int distance2_mm = 0;
      leftForwards();
      rightForwards();
      distance_mm = sonar_mm(); //call the function sonar_mm and store the result in distance_mm
      distance2_mm = sonar2_mm();
      Serial.print("Distance Front="); //print the result to the serial monitor
      Serial.println(distance_mm);
      Serial.print("Distance Back="); //print the result to the serial monitor
      Serial.println(distance2_mm);
      int sensorVal = digitalRead(A5);
      Serial.println(sensorVal);


        if (distance_mm>100)
  
        { 
                w1 = analogRead(A1);
                w2 = analogRead(A2);
                w3 = analogRead(A3);
                w4 = analogRead(A5);
              
                num = (w1 * x1) + (w2 * x2) + (w3 * x3) + (w4 * x4);
                den = w1 + w2 + w3 + w4;
                
                float lineDist = num/den;  
              
                leftForwards();
                rightForwards();
                Serial.print("Distance from line = ");
                Serial.println(lineDist);
          if lineDist >= 15 {
                analogWrite(5, 70); //left
                analogWrite(6, 10); //right
          }
          else if (lineDist < 15 && lineDist >= 7) {
                analogWrite(5, 65);
                analogWrite(6, 40);
          }
          }
   
          else if(distance_mm < 100)
         {
              leftBackwards();
              rightBackwards();
              analogWrite(5,0);
              analogWrite(6,0);
             
              unsigned int store_distance=500;
              int angle_at_shortest_distance;
              Serial.print("Distance Back="); //print the result to the serial monitor
              Serial.println(distance2_mm);
              for(angle2 = 0; angle2 < 180; angle2=angle2+20)
              {
                      distance2_mm=sonar2_mm();                      
                      Serial.print("Distance Back="); //print the result to the serial monitor
                      Serial.println(distance2_mm);
                      servo2.write(angle2);
                      delay(500);
                      if(distance2_mm<store_distance)
                      {
                        store_distance=distance2_mm;
                        angle_at_shortest_distance=angle2;
                      }
              }
              angle2=angle_at_shortest_distance;
              servo2.write(angle2);
                    
                    if(angle_at_shortest_distance<90)
                      { 
                     
                          distance2_mm=sonar2_mm();                      
                          Serial.print("Distance Back="); //print the result to the serial monitor
                          Serial.println(distance2_mm);
                          if(distance2_mm<50)
                          {
                            leftBackwards();
                            rightBackwards();
                            analogWrite(5,0);
                            analogWrite(6,0);
                            break;
                          }
                          else if(distance2_mm>50)
                          {
                            leftBackwards();
                            rightForwards();
                            analogWrite(5,100);
                            analogWrite(6,100);
                            delay(1000);
                          }
                     
                      }
                      else if(angle_at_shortest_distance>90)
                      { 
                        
                           
                          distance2_mm=sonar2_mm();                      
                          Serial.print("Distance Back="); //print the result to the serial monitor
                          Serial.println(distance2_mm);
                          if(distance2_mm<50)
                          {
                            leftBackwards();
                            rightBackwards();
                            analogWrite(5,0);
                            analogWrite(6,0);
                            break;
                          }
                          else if(distance2_mm>50)
                          {
                            leftForwards();
                            rightBackwards();
                            analogWrite(5,100);
                            analogWrite(6,100);
                            delay(1000);
                          }
                      }
                 
            
                      else
                       {
                          
                            leftBackwards();
                            rightForwards();
                            analogWrite(5,100);
                            analogWrite(6,100);
                            delay(1000);  
                       }
              
          }
            servo2.write(angle2);
            servo.write(angle);               
            delay(15);                   
      } 
  // now scan back from 180 to 0 degrees
  for(angle = 180; angle > 0; angle=angle-10)    
        { 
        unsigned int distance_mm = 0; //This variable will hold the distance
        unsigned int distance2_mm = 0;
        leftForwards();
        rightForwards();
        distance_mm = sonar_mm(); //call the function sonar_mm and store the result in distance_mm
        distance2_mm = sonar2_mm();
        Serial.print("Distance Front="); //print the result to the serial monitor
        Serial.println(distance_mm);
        Serial.print("Distance Back="); //print the result to the serial monitor
        Serial.println(distance2_mm);
        int sensorVal = digitalRead(A5);
        Serial.println(sensorVal);

        
        
          if (distance_mm > 100)
  
                { 
                  w1 = analogRead(A1);
                  w2 = analogRead(A2);
                  w3 = analogRead(A3);
                  w4 = analogRead(A5);
                
                  num = (w1 * x1) + (w2 * x2) + (w3 * x3) + (w4 * x4);
                  den = w1 + w2 + w3 + w4;
                  
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
                  if(distance_mm <= 100)
                  {
                      analogWrite(5,0);  
                      analogWrite(6,0);
                  }
                }
          else if(distance_mm < 100)
         {
              leftBackwards();
              rightBackwards();
              analogWrite(5,0);
              analogWrite(6,0);
              
              unsigned int store_distance=500;
              int angle_at_shortest_distance;
              Serial.print("Distance Back="); //print the result to the serial monitor
              Serial.println(distance2_mm);
              for(angle2 = 0; angle2 < 180; angle2=angle2+20)
              {
                      distance2_mm=sonar2_mm();
                      Serial.print("Distance Back="); //print the result to the serial monitor
                      Serial.println(distance2_mm);
                      delay(500);
                      servo2.write(angle2);
                      if(distance2_mm<store_distance)
                      {
                        store_distance=distance2_mm;
                        angle_at_shortest_distance=angle2;
                      }
              }
              angle2=angle_at_shortest_distance;
              servo2.write(angle2);
             
                    if(angle_at_shortest_distance<90)
                      { 
                        
                          
                          distance2_mm=sonar2_mm();                      
                          Serial.print("Distance Back="); //print the result to the serial monitor
                          Serial.println(distance2_mm);
                          if(distance2_mm<50)
                          {
                            leftBackwards();
                            rightBackwards();
                            analogWrite(5,0);
                            analogWrite(6,0);
                            break;
                          }
                          else if(distance2_mm>50)
                          {
                            leftBackwards();
                            rightForwards();
                            analogWrite(5,100);
                            analogWrite(6,100);
                            delay(1000);
                          }
                        
                            
                      }
                      if(angle_at_shortest_distance>90)
                      { 
                       
                          
                          distance2_mm=sonar2_mm();                      
                          Serial.print("Distance Back="); //print the result to the serial monitor
                          Serial.println(distance2_mm);
                          if(distance2_mm<50)
                          {
                            leftBackwards();
                            rightBackwards();
                            analogWrite(5,0);
                            analogWrite(6,0);
                            break;
                          }
                          else if(distance2_mm>50)
                          {
                            leftForwards();
                            rightBackwards();
                            analogWrite(5,100);
                            analogWrite(6,100);
                            delay(1000);
                          }
                        
                        
                      }
                      else
                        {
                            leftForwards();
                            rightBackwards();
                            analogWrite(5,100);
                            analogWrite(6,100);
                            delay(1000);
                        }
         }
              servo2.write(angle2);                                 
              servo.write(angle);           
              delay(15);       
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


  
unsigned int sonar_mm(void){
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

unsigned int sonar2_mm(void){
  long duration = 0;
  const float speed_sound = 340.29;// m/s, "const" makes the compiler able to optimise the program where this variable is used, cool!
  // Read in a distance from the ultrasonic distance sensor:
  // The ultrasonic burst is triggered by a HIGH pulse of 10 microseconds.
  digitalWrite(TRIG1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG1, LOW);
  //read length of time pulse
  duration = pulseIn(ECHO1, HIGH); //This function measures a pulsewidth and returns the width in microseconds
  // convert the time into a distance
  // the code "(unsigned int)" turns the result of the distance calculation
  // into an integer instead of a floating point (decimal or fractional) number.
  return (unsigned int)(0.5 * duration * 1e-6 * speed_sound * 1e3); 
  //"unsigned" ensures we are returning an unsigned number, remember that there is no such thing as negative distance.
  
}
