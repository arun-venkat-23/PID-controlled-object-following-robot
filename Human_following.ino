#include <NewPing.h>
#include <Servo.h>

#define ULTRASONIC_SENSOR_TRIG 11
#define ULTRASONIC_SENSOR_ECHO 12

#define MAX_FORWARD_MOTOR_SPEED 75
#define MAX_MOTOR_TURN_SPEED_ADJUSTMENT 50

#define MIN_DISTANCE 10
#define MAX_DISTANCE 30

#define IR_SENSOR_RIGHT 2
#define IR_SENSOR_LEFT 3

int posY = 90;
Servo Y;

//Right motor
int enableRightMotor=5;
int rightMotorPin1=7;
int rightMotorPin2=8;

//Left motor
int enableLeftMotor=6;
int leftMotorPin1=9;
int leftMotorPin2=10;

NewPing mySensor(ULTRASONIC_SENSOR_TRIG, ULTRASONIC_SENSOR_ECHO, 400);

void setup()
{
  // put your setup code here, to run once:
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
   Y.attach(13);

  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  rotateMotor(0,0);   
}


void loop()
{
  int distance = mySensor.ping_cm();
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  //NOTE: If IR sensor detects the hand then its value will be LOW else the value will be HIGH
  
  //If right sensor detects hand, then turn right. We increase left motor speed and decrease the right motor speed to turn towards right
  if (leftIRSensorValue == LOW && rightIRSensorValue == HIGH )
  {
      rotateMotor(MAX_FORWARD_MOTOR_SPEED - MAX_MOTOR_TURN_SPEED_ADJUSTMENT, MAX_FORWARD_MOTOR_SPEED + MAX_MOTOR_TURN_SPEED_ADJUSTMENT ); 
  }
  //If left sensor detects hand, then turn left. We increase right motor speed and decrease the left motor speed to turn towards left
  else if (leftIRSensorValue == HIGH && rightIRSensorValue == LOW )
  {
      rotateMotor(MAX_FORWARD_MOTOR_SPEED + MAX_MOTOR_TURN_SPEED_ADJUSTMENT, MAX_FORWARD_MOTOR_SPEED - MAX_MOTOR_TURN_SPEED_ADJUSTMENT); 
  }
  //If distance is between min and max then go straight
  else if (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE)
  {
    rotateMotor(MAX_FORWARD_MOTOR_SPEED, MAX_FORWARD_MOTOR_SPEED);
  }
  //stop the motors
  else 
  {
    rotateMotor(0, 0);
  }
 if(digitalRead(IR_SENSOR_LEFT) == 1 && digitalRead(IR_SENSOR_RIGHT) == 0)
  {
    while(digitalRead(IR_SENSOR_LEFT) != 0 && digitalRead(IR_SENSOR_RIGHT) != 1)
    {
      if(posY == 50)
      {break;}
      posY--;
      Y.write(posY);
      delay(15);
    }
  }
  else if(digitalRead(IR_SENSOR_LEFT) == 0 && digitalRead(IR_SENSOR_RIGHT) == 1)
  {
    while(digitalRead(IR_SENSOR_LEFT) != 1 && digitalRead(IR_SENSOR_RIGHT) != 0)
    {
      if(posY == 160)
      {break;}
      posY++;
      Y.write(posY);
      delay(15);
    }
  }
}


void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else 
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}
