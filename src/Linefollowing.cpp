#include <Arduino.h>
#include <Romi32U4.h>
#include "Romi32U4Motors.h"


// VARIABLES

const int lineRight = 20;
const int lineLeft = 21;

const int whiteLine = 600;

// STATS

const float track = 5.78;
const float diameter = 2.75;

const float defaultSpeed = 100;   //was 360
const float slowSpeed = 120;

// FUNCTIONS----------------------------------------------------------------------------


// TIMER---------------------------------------------------------------------------------

bool timeUpdateCheck(int samplePeriod) {    //mock delay() function this returns bool and allows me to change sample period for each function its called in
  unsigned long beginTime = millis();                 //after the elapsed time is equal to the sample period, used often
  unsigned long currentTime = millis();
  while (samplePeriod >= currentTime - beginTime) {
    currentTime = millis();     //updates so that current time will get bigger every time its checked in the while loop
  }
  return true; //it only returns true after time is elapsed, can not return false
}//Matt

// LINE FOLLOWING GROUP---------------------------------------------------------------------


float piControl(int target, int actual, float Kp, float Ki) {     //proportional and intergral control to assist line following
  static float errorSum = 0;                                      //we did not end up using integral control, but if we included
  static float lastTime = 0;                                      //it here and did not need it then we could easily set ki to 0
  static float effort = 0;
  float samplePeriod = 50;
  static float maxSum = 2000;
  unsigned long now = millis();
  if (now - lastTime >= samplePeriod) {
    float error = target - actual;   //error
    errorSum += error ;                //Intergral Term
    if (errorSum > maxSum) errorSum = maxSum;
    if (errorSum < -maxSum) errorSum = -maxSum;


    effort = Kp * error + Ki * errorSum;    //Calculation for Pi effort
    lastTime = now;
    return effort;
  }
  return effort;
}//Matt

void followLine(LeftMotor motorLeft, RightMotor motorRight) {
  static float Kp = .07; // even this low there is occasional oscillation
  static float Ki = 0;
  static int targetADC = 30; // total reflection

  int left = analogRead(lineLeft);
  int right = analogRead(lineRight);
  Serial.print(left);
  Serial.print("  ");
  Serial.println(right);

  float motorEffort = piControl(targetADC, (left - right), Kp, Ki); // - means turn left, + means turn right

  if (motorEffort != 0) { // turn left
    motorLeft.setMotorEffort(-(defaultSpeed + motorEffort)); // left forward and right backwards simultaneously for better error correction
    motorRight.setMotorEffort(-(defaultSpeed - motorEffort));
  }else { // go straight
    motorLeft.setMotorEffort(-defaultSpeed);
    motorRight.setMotorEffort(-defaultSpeed);
  }
}

//TURN HELPERS----------------------------------------------------------------------------

bool turnLeftHelper() {                                       //checks if right element detects line, compares to prev value, name convention since this is used in turnLeft()
  bool returnValue = false;
  static bool previousRight = true;
  bool currentRight = false;
  int right = analogRead(lineRight);
  if (right < whiteLine) {
    currentRight = true;
  }
  if (currentRight == true && previousRight == false) {
    returnValue = true;
  }
  previousRight = currentRight;
  return returnValue;
}//Matt


bool turnRightHelper() {                     //checks if left element detects line, compares to prev value, name convention since this is used in turnRight()
  bool returnValue = false;
  static bool previousLeft = true;
  bool currentLeft = false;
  int left = analogRead(lineLeft);
  if (left < whiteLine) {
    currentLeft = true;
  }
  if (currentLeft == true && previousLeft == false) {
    returnValue = true;
  }
  previousLeft = currentLeft;
  return returnValue;
}//Matt


bool reverseDirectionHelper() {              //exact same as turnLeftHelper, different name to help comprehension between team members
  bool returnValue = false;
  static bool previousRight = true;
  bool currentRight = false;
  int right = analogRead(lineRight);
  if (right < whiteLine) {
    currentRight = true;
  }
  if (currentRight == true && previousRight == false) {
    returnValue = true;
  }
  previousRight = currentRight;
  return returnValue;
}//Matt

// LINE DETECTION GROUP-----------------------------------------------------------------

bool lineDetectedRight() { // detects line on right sensor
  bool returnValue = false;
  int right = analogRead(lineRight);
  Serial.print(right);
  Serial.print("  ");
  if (right < whiteLine) {
    returnValue = true;
  }
  return returnValue;
}//Martin

bool lineDetectedLeft() { // detects line on left sensor
  bool returnValue = false;
  int left = analogRead(lineLeft);
  Serial.println(left);
  if (left < whiteLine){
    returnValue = true;
  }
  return returnValue;
}//Martin

bool moveUntilLine(LeftMotor motorLeft, RightMotor motorRight) {    //romi runs until either the left or right sensor reads a line
  if(lineDetectedRight()){
    motorLeft.setMotorEffort(0);
  }
  else{
    motorLeft.setMotorEffort(-50);
  }
  if(lineDetectedLeft()){
    motorRight.setMotorEffort(0);
  }
  else{
    motorRight.setMotorEffort(-50);
  }
  if(lineDetectedLeft() && lineDetectedRight()){
    return true;
  }
  else{
    return false;
  }
}//Matt

// TURNING GROUP ------------------------------------------------------------------------------------
//basic premise of turning functions is that they will run motors at some speed (depending how it wants to turn)
//and when it runs, it checks to see if a sensor is on a line. For example, to turn left, the right sensor element "catches" the line, 
//allowing the romi to be on the line to continue line following


void turnLeft(int samplePeriod, LeftMotor motorLeft, RightMotor motorRight) {         //turns left by setting right motor to default speed and left motor to zero
  motorRight.setMotorEffort(0);      //this alligns line sensor with line
  motorLeft.setMotorEffort(-defaultSpeed);               
  timeUpdateCheck(samplePeriod);
  while (!turnLeftHelper()) {}
  motorRight.setMotorEffort(0);
  motorLeft.setMotorEffort(0);
}//Matt

void turnRight(int samplePeriod, LeftMotor motorLeft, RightMotor motorRight) {        //turns right by setting left motor to default speed and right motor to zero
  motorRight.setMotorEffort(-defaultSpeed);                 //this alligns line sensor with line
  motorLeft.setMotorEffort(0);
  Serial.println("start wait");
  timeUpdateCheck(samplePeriod);
  Serial.println("end waited");
  while (!turnRightHelper()) {}
  motorRight.setMotorEffort(0);
  motorLeft.setMotorEffort(0);
}//Matt

void reverseDirection(int samplePeriod, LeftMotor motorLeft, RightMotor motorRight) { //rotates around center axis CCW
  motorLeft.setMotorEffort(defaultSpeed / 2);
  motorRight.setMotorEffort(-defaultSpeed / 2);
  timeUpdateCheck(samplePeriod);
  while (!turnRightHelper()) {}
  motorRight.setMotorEffort(0);
  motorLeft.setMotorEffort(0);
}//Matt

void turnRightReverse(int samplePeriod, LeftMotor motorLeft, RightMotor motorRight) {   //tunrns CW 90 degrees by setting right motor to negative default speed and left motor to zero
  motorRight.setMotorEffort(0);       //this alligns line sensor perpendicular to line
  motorLeft.setMotorEffort(defaultSpeed);
  timeUpdateCheck(samplePeriod);
  while (!turnLeftHelper()) {}
  motorRight.setMotorEffort(0);
  motorLeft.setMotorEffort(0);
}//Matt

void turnLeftReverse(int samplePeriod, LeftMotor motorLeft, RightMotor motorRight) {   //tunrns CCW 90 degrees by setting right motor to zero and left motor to negative default speed 
  motorRight.setMotorEffort(defaultSpeed);                  //this alligns line sensor perpendicular to line
  motorLeft.setMotorEffort(0);
  timeUpdateCheck(samplePeriod);
  while (!turnLeftHelper()) {}
  motorRight.setMotorEffort(0);
  motorLeft.setMotorEffort(0);
}//Matt

void turn(int turnCount, LeftMotor motorLeft, RightMotor motorRight, int instructions[9]) {
  if (instructions[turnCount] == 0) {        //left, only power to right motor
    turnLeft(400, motorLeft, motorRight);
    return;
  } else if (instructions[turnCount] == 2) {  //right, only power to left motor
    turnRight(400, motorLeft, motorRight);
    return;
  } else if (instructions[turnCount] == 3) {  //around CCW, half negative power to left motor, half positive power to right motor
    reverseDirection(600, motorLeft, motorRight);
    timeUpdateCheck(100);
    return;
  } else if (instructions[turnCount] == 4) {  //K turn, CCW
    turnRightReverse(400, motorLeft, motorRight);                             //negative speed to right motor until right reads line, then turn right till line
    turnRight(400, motorLeft, motorRight);                                    //this allows us to separate a consistent distance away from bags and delivery zones
    motorRight.setMotorEffort(-defaultSpeed);                            
    motorLeft.setMotorEffort(0);
    timeUpdateCheck(120);               //slight dead reckoning, would work perfect if line had no thickness, but since it does, the robot will not make exact 180 turn, more like 175
    motorRight.setMotorEffort(0);
    motorLeft.setMotorEffort(0);
    timeUpdateCheck(100);
    return;
  }
  else if (instructions[turnCount] == 5) {  //K turn, CW, not used, written in case testing might require opposite k turn
    turnLeftReverse(400, motorLeft, motorRight);                            //negative speed to left until left reads line, then turn left till line
    turnLeft(400, motorLeft, motorRight);
    return;
  } else if (instructions[turnCount] == 6) {  //turn right 90, negative speed to right motor until left sensor detects line
    turnRightReverse(400, motorLeft,  motorRight);
    return;
  } else if (instructions[turnCount] == 1) {  //straight
    motorLeft.setMotorEffort(-defaultSpeed);
    motorRight.setMotorEffort(-defaultSpeed);
    timeUpdateCheck(120);               //skip over line, since only called at 4 point intersection it had to drive over the intersection without detecting it again
    motorLeft.setMotorEffort(0);              //dead reckoning necessary since, depending on surface, speed, and presence of bag, momentum could carry romi over intersection, meaning a detect intersection gone function wouldnt always work
    motorRight.setMotorEffort(0);
    return;
  } else if (instructions[turnCount] == 6) { //since this will never run without an intersection detected, this isn't needed but its there just in case
    return;
  }
}//Matt

// INTERSECTION GROUP--------------------------------------------------------------------------------------------------

// INTERSECTION GROUP

bool intersectionDetected(int samplePeriod) { // takes in a sample period
  bool returnValue = false;
  static bool previousInt = false;
  bool currentInt = false;
  int left = analogRead(lineLeft);
  int right = analogRead(lineRight);
  if (left < whiteLine && right < whiteLine) { // if both are over tape
    Serial.println("Place 1");
    currentInt = true;
  }
  if (currentInt && !previousInt) { // new intersection detected
    // if (timeUpdateCheck(samplePeriod)) {
    //   left = analogRead(lineLeft);
    //   right = analogRead(lineRight);
    Serial.println("Place 2");
      if (left < whiteLine && right < whiteLine) {
        Serial.println("Place 3");
        returnValue = true;
      }
    // }
  }
  previousInt = currentInt;
  return returnValue;
}//Martin