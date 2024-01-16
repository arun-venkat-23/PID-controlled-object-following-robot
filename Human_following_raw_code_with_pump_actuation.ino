#include "TimerOne.h"
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

int Motor1APin=31; 
int Motor1BPin=33; 
int Motor2APin=35; 
int Motor2BPin=37; 
int motor_speed_Pin1 = 2; 
int motor_speed_Pin2 = 3; 
int FEchoPin=49; // Forward ultrasonic sensor echo
int FTrigPin=48; // Forward ultrasonic sensor trigger
int REchoPin=46; // Right ultrasonic sensor echo
int RTrigPin=47; // Right ultrasonic sensor trigger
int LEchoPin=51; // Left ultrasonic sensor echo
int LTrigPin=50; // Left ultrasonic sensor trigger
int IRSensPin=A1; // IR sensor data
int distanceFAR = 60; // set max distance to enable search function
int pumpPin1 = 8;
int pumpPin2 = 9;
float Fdistance;
float Rdistance;
float Ldistance;
float FdistanceIR;
float FdistanceLOCAL;
float distance;
float angle;
float sensorangle = 25;
float temp1;
float temp2;
bool over_cup = false;
bool selectionMade = false;
char directionChar = 'Z';
float Kp = 13;
float Ki = 0.9;
float Kd = 9.2;
float P;
float I;
float D;
int motorspeedAdj = 0;
int motorSpeed = 0;
float prev_Error = 0;
float error;
float motor_pulse = 0;
int aligned = 0;
int pinhigh = 0;
float RpwmMod = 0;
float LpwmMod = 0;
int selection = 0;
bool PumpsCompleted = false;
float offtimer = 0;
float ontimer = 0;
int selectionPin1 = A5;
int selectionPin2 = A7;
int selectionPin3 = A6;
int previousstate = 0;
unsigned long timeBegin; //time vars for dispensing
unsigned long timeEnd;
int timeOn = 0;
int pumpTime = 5000;
void setup() {
Serial.begin(9600); // Initialize serial com
pinMode(Motor1APin,OUTPUT);
pinMode(Motor1BPin,OUTPUT);
pinMode(Motor2APin,OUTPUT);
pinMode(Motor2BPin,OUTPUT);
pinMode(FTrigPin,OUTPUT);
pinMode(RTrigPin,OUTPUT);
pinMode(LTrigPin,OUTPUT);
pinMode(FEchoPin,INPUT);
pinMode(REchoPin,INPUT);
pinMode(LEchoPin,INPUT);
pinMode(IRSensPin,INPUT);
sensorangle = sensorangle * 2 * PI / 360;
}
void find_Fdistance() // Measure the distance from forward ultrasonic sensor
{
temp1 = 0;
for(int i = 0; i<5; i++){ // take 5 measurements for average
digitalWrite(FTrigPin, LOW); // For low voltage 2 us ultrasonic launch
delayMicroseconds(2);
digitalWrite(FTrigPin, HIGH); // trig 10us high voltage
delayMicroseconds(10);
digitalWrite(FTrigPin, LOW); // Maintaining low voltage ultrasonic launch
Fdistance = pulseIn(FEchoPin, HIGH); // Read the time difference
Fdistance = (Fdistance/58); // output distance in cm
temp1 += Fdistance; }
Fdistance = temp1/5;
Serial.print("F-distance Sonar:"); //The output distance (unit: cm)
Serial.println(Fdistance); //According to the distance
return; }
float find_FdistanceIR() { // Measure the distance from forward IR sensor
temp1 = analogRead(IRSensPin); //obtain 3 measurements and take average
temp2 = analogRead(IRSensPin);
FdistanceLOCAL=(4271*float(pow((temp1+temp2+analogRead(IRSensPin))/3,(-1.12))));
if (FdistanceLOCAL>65){ //sensor max is 80cm, limit 65 to ignore other objects
FdistanceLOCAL = 65;}
Serial.print("F-distance IR:");
Serial.println(FdistanceLOCAL);
//Serial.print("Value read IR:");
//Serial.println(temp);
return FdistanceLOCAL;
}
void find_Rdistance(){ // Measure the distance from right sensor
temp1 = 0;
for(int i = 0; i<5; i++){
digitalWrite(RTrigPin, LOW); // For low voltage 2 us ultrasonic launch
delayMicroseconds(2);
digitalWrite(RTrigPin, HIGH); //set trig High for 10us
delayMicroseconds(10);
digitalWrite(RTrigPin, LOW); // Maintaining low voltage ultrasonic launch
Rdistance = pulseIn(REchoPin, HIGH); // Read the time difference
Rdistance = (Rdistance/58); // distance in cm
temp1 += Rdistance;
}
Rdistance = temp1/5;
Serial.print("R-distance Sonar:"); //The output distance (unit: cm)
Serial.println(Rdistance); //According to the distance
return;
}
void find_Ldistance() // Measure the distance from left sensor
{
temp1 = 0;
for(int i = 0; i<5; i++){
digitalWrite(LTrigPin, LOW); // For low voltage 2 us ultrasonic launch
delayMicroseconds(2);
digitalWrite(LTrigPin, HIGH); // set trig HIGH for 10us
delayMicroseconds(10);
digitalWrite(LTrigPin, LOW); // Maintaining low voltage ultrasonic launch
Ldistance = pulseIn(LEchoPin, HIGH); // Read the time difference
Ldistance = (Ldistance/58); // distance in cm
temp1 += Ldistance;
}
Ldistance = temp1/5;
Serial.print("L-distance Sonar:"); //The output distance (unit: cm)
Serial.println(Ldistance); //According to the distance
return;
}
void DriveMotors(char directionChar){
switch(directionChar){
case 'F': //Forward
digitalWrite(Motor1APin, LOW);
digitalWrite(Motor1BPin, HIGH);
digitalWrite(Motor2APin, HIGH);
digitalWrite(Motor2BPin, LOW);
break;
case 'B': // Reverse
digitalWrite(Motor1APin, HIGH);
digitalWrite(Motor1BPin, LOW);
digitalWrite(Motor2APin, LOW);
digitalWrite(Motor2BPin, HIGH);
break;
case 'R': //rotate right
digitalWrite(Motor1APin, LOW);
digitalWrite(Motor1BPin, HIGH);
digitalWrite(Motor2APin, LOW);
digitalWrite(Motor2BPin, HIGH);
break;
case 'L': //rotate left
digitalWrite(Motor1APin, HIGH);
digitalWrite(Motor1BPin, LOW);
digitalWrite(Motor2APin, HIGH);
digitalWrite(Motor2BPin, LOW);
break;
case 'S': //stop the motors
digitalWrite(Motor1APin, LOW);
digitalWrite(Motor1BPin, LOW);
digitalWrite(Motor2APin, LOW);
digitalWrite(Motor2BPin, LOW);
analogWrite(motor_speed_Pin1, 0);
analogWrite(motor_speed_Pin2, 0);
break;
default: break;
}
return; }
void Search(){
DriveMotors('S');
analogWrite(motor_speed_Pin1, 140);
analogWrite(motor_speed_Pin2, 140);
delay(100); //wait for sensor to settle
find_Rdistance();
find_Ldistance();
//Turn Left
if(Rdistance<100&&((Rdistance<(Ldistance+5)) || (Rdistance<(Ldistance-5)) )){
DriveMotors('L');
while(FdistanceIR > 60){
FdistanceIR = find_FdistanceIR();
}
DriveMotors('S');
return;
}
//Turn Right
else if(Ldistance<100 && ((Ldistance < (Rdistance+5)) || ((Ldistance) <
(Rdistance-5))) ){
DriveMotors('R');
while(FdistanceIR > 60){
//Serial.println("inside drive right control"); //Boss debug
//Serial.println(Ldistance);
FdistanceIR = find_FdistanceIR();
}
DriveMotors('S');
return;
}
else{
DriveMotors('R');
while(FdistanceIR > 50){
analogWrite(motor_speed_Pin1, 130);
analogWrite(motor_speed_Pin2, 130);
FdistanceIR = find_FdistanceIR();
}
DriveMotors('S');
return;
}
}
void alignment(){
analogWrite(motor_speed_Pin1, 80);
analogWrite(motor_speed_Pin2, 80);
//adjust right
if(Rdistance>(Ldistance + 10)){
//Serial.println("adjustment RIGHT"); //Boss debug
DriveMotors('R');
delay(200);
DriveMotors('S');
}
//adjust left
else if(Ldistance>(Rdistance + 10)){
//Serial.println("adjustment LEFT"); //Boss debug
DriveMotors('L');
delay(200);
DriveMotors('S');
}
}
void Finealignment(){
//adjust left
if((Ldistance<10)&&(Ldistance<(Rdistance-2))){
//Serial.println("adjustment RIGHT"); //Boss debug
DriveMotors('S');
analogWrite(motor_speed_Pin1, 150);
analogWrite(motor_speed_Pin2, 150);
DriveMotors('B');
DriveMotors('S');
analogWrite(motor_speed_Pin1, 120);
analogWrite(motor_speed_Pin2, 120);
DriveMotors('L');
delay(175);
DriveMotors('S');
}
//adjust right
else if((Rdistance<10)&&(Rdistance<(Ldistance-2))){
//Serial.println("adjustment LEFT"); //Boss debug
DriveMotors('S');
analogWrite(motor_speed_Pin1, 150);
analogWrite(motor_speed_Pin2, 150);
DriveMotors('B');
DriveMotors('S');
analogWrite(motor_speed_Pin1, 120);
analogWrite(motor_speed_Pin2, 120);
DriveMotors('R');
delay(175);
DriveMotors('S');
}
//adjust back
else if((Rdistance<5)&&(Fdistance>1000)&&(FdistanceIR<5.5)&&(Ldistance<5)){
analogWrite(motor_speed_Pin1, 150);
analogWrite(motor_speed_Pin2, 150);
DriveMotors('R');
delay(175);
DriveMotors('S');
}
}
void PID(){
error = FdistanceIR - 5.2; //the desired position of the cup;note sensor min is 5cm
P = error;
I = I + error;
D = error - prev_Error;
Serial.print("POSITION ERROR: "); //Boss debug
Serial.println(error); //Boss debug
prev_Error = error;
if(error<0){ //overshoot, reverse slightly
analogWrite(motor_speed_Pin1, 200);
analogWrite(motor_speed_Pin2, 200);
DriveMotors('B');
delay(100);
DriveMotors('S');
return;
}
motorspeedAdj = P*Kp + I*Ki + D*Kd;
if((motorspeedAdj + motorSpeed) > 255){ //adjustment cannot exceed PWM 255 max
Serial.println("Speed adjust greater than 255"); //Boss debug
analogWrite(motor_speed_Pin1, 255);
analogWrite(motor_speed_Pin2, 255);
DriveMotors('F');
}
else if( ((motorspeedAdj+motorSpeed)<255) && ((motorspeedAdj+motorSpeed)>100) ){
Serial.println("100<Speed adjust<255"); //Boss debug
analogWrite(motor_speed_Pin1, motorspeedAdj + motorSpeed);
analogWrite(motor_speed_Pin2, motorspeedAdj + motorSpeed);
DriveMotors('F');
}
else{
analogWrite(motor_speed_Pin1, 125); //the min PWM the motors can operate at
analogWrite(motor_speed_Pin2, 125); //with the load is ~125 in our case
DriveMotors('F');
}
if(error<0.75){DriveMotors('S');}
Serial.print("SPEED | PID: "); //Boss debug
Serial.println(motorspeedAdj + motorSpeed); //Boss debug
}
void selectionButton(){
while(!selectionMade){
if (analogRead(selectionPin1) > 1000){
selection = 1;
selectionMade = true;
}
else if (analogRead(selectionPin2) > 1000){
selection = 2;
selectionMade = true;
}
else if (analogRead(selectionPin3) > 1000){
selection = 3;
selectionMade = true;
}
else {
selection = 0;
}
}
}
void checkOverCup(){
find_Fdistance();
find_Rdistance();
find_Ldistance();
FdistanceIR=find_FdistanceIR();
if(Fdistance<8 && Ldistance<8 && Rdistance<8 && FdistanceIR<6.2){
over_cup = true;
}
else{
over_cup = false;
}
}
void Pumps(){
timeBegin = millis();
if (selection == 1){
//Serial.println(1);
analogWrite(pumpPin1,255);
analogWrite(pumpPin2,0);
}
else if (selection == 2){
//Serial.println(2);
analogWrite(pumpPin1,0);
analogWrite(pumpPin2,255);
}
else if (selection == 3){
//Serial.println(3);
analogWrite(pumpPin1,127);
analogWrite(pumpPin2,127);
}
while(over_cup){
timeEnd = millis();
checkOverCup();
Serial.println("bf the if"); //Boss debug
if((timeEnd-timeBegin)>pumpTime){
selectionMade = false;
pumpTime = 5000;
analogWrite(pumpPin1,0);
analogWrite(pumpPin2,0);
return;
}
}
analogWrite(pumpPin1,0);
analogWrite(pumpPin2,0);
timeEnd = millis();
pumpTime -= (timeEnd-timeBegin);
}
void loop() {
if(!selectionMade){ //enter selection function to wait for a selection to be made
DriveMotors('S');
selectionButton(); //wait here till selection is made.
}
checkOverCup(); //once a selection is made, continously check if the dispenser is
find_Fdistance(); //over the cup
find_Rdistance();
find_Ldistance();
FdistanceIR=find_FdistanceIR();
if(!over_cup){
if(FdistanceIR>50 || Fdistance>100){
Serial.println("Search Enabled");
motorspeedAdj = 0; //since search has been enabled, reset PID so it will not
P = 0; // accumate error while finding the cup around the robot
I = 0;
D = 0;
Search();
}
if((Rdistance>(Ldistance + 10)) || (Ldistance>(Rdistance + 10) )){
//Serial.println("adjustment needed"); //Boss debug
motorspeedAdj = 0;
alignment();
}
if((FdistanceIR>3.5)&&(((Rdistance<10)&&(Rdistance<(Ldistance-2))) ||
((Ldistance<10)&&(Ldistance<(Rdistance-2)))) || ((Fdistance>1000)&&(FdistanceIR<5.5))
){
Serial.println("FINE adjustment needed"); //Boss debug
Finealignment();
}
PID();
}
else if(over_cup){
Pumps();
motorspeedAdj = 0;
P = 0;
I = 0;
D = 0;
DriveMotors('S');
}
}