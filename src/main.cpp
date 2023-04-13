#include <Arduino.h>
#include <Romi32U4.h>
#include "BlueMotor.h"
#include <servo32u4.h>
#include "claw.h"
#include "IRdecoder.h"
#include "remoteconstants.h"
#include "Linefollowing.h"
#include "Romi32U4Motors.h"
#include "Rangefinder.h"
#include "Chassis.h"

BlueMotor motor;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Servo32U4Pin5 jawServo;
IRDecoder decoder(14);
LeftMotor motorLeft;
RightMotor motorRight;
Rangefinder rangefinder(17, 12);
Chassis chassis;

// line following variables
int motorEffort = 400;

// ultrasonic sensor variables
int position1 = 15; // building 25 degree
int position3 = 16; // building 45 degree
int position2 = 27; // stop location
int position4 = 7;  // low block
int position7 = 15; // other side of course end

// where we want the blue motor
int clearance = 2000;
int twentyFive = 1000;
int fourtyFive = 816;
int flat = 0;

// LOGIC VARIABLES
// 0 = left, 1 = straight, 2 = right, 3 = around CCW, 4 = K turn CCW, 5 = K turn CC, 6 = turn 90 right, 7 = blank
int turnDirectionsOne[9] = {3, 0, 3, 2, 3, 0, 0, 0, 0}; // array for demo, has correct values
int turnDirectionsTwo[9] = {3, 2, 3, 1, 3, 2, 2, 2, 2};

// Tells us the state of the machine
int turncount = 0;
int prevLocationWanted = 0;
int locationWanted = 2;
bool openPosition = true;
bool replacementComplete = false;
int step = 0;

int ultrasonicReading;

int16_t code;

enum ROBOT_STATE
{
  WAITING,
  PATHA,
  PATHB
};

enum COURSE
{
  SERVO,
  BLUE_MOTOR,
  LINE_FOLLOWING,
  TURNING,
  PAUSING,
  MOVE_TO_LINE
};

ROBOT_STATE robotState = WAITING;
COURSE courseState = LINE_FOLLOWING;

void ISRsa()
{
  if (robotState == PATHA)
  {                                             // if using the non-continuous servo
    jawServo.writeMicroseconds(analogRead(A0)); // stop in place
  }
  if (robotState == PATHB)
  {                                // if using the continuous servo
    jawServo.writeMicroseconds(0); // stop in place
  }
  motorLeft.setMotorEffort(0); // stop moving
  motorRight.setMotorEffort(0);
  while (1 == 1)
  { // stay here until
    if (decoder.getKeyCode(remote0))
    { // remotely brought out of the while loop
      break;
    }
  }
}

void setup()
{
  Serial.begin(9600);
  attachInterrupt(decoder.getKeyCode(remoteStopMode), ISRsa, RISING);
  chassis.init();
  motor.setup();
  motor.reset();
  jawServo.setMinMaxMicroseconds(0, 20000);
  decoder.init();
  motorLeft.init();
  motorRight.init();
  rangefinder.init();
  delay(3000);
  Serial.println("ON");
}

// void loop(){ //When button A is pressed, open. When button B is pressed, close. If stuck, open.
//   if(buttonA.isPressed()){
//     open(jawServo);
//   }

//   if(buttonB.isPressed()){
//     close(jawServo);
//   }
// }

int ultrasonicReturn(Rangefinder rangefinder)
{
  int arr[5];
  int n = 5;
  int min, temp;
  for (int k = 0; k < n - 1; k++)
  {
    arr[k] = rangefinder.getDistance();
    delay(100);
  }
  for (int i = 0; i < n - 1; i++)
  {
    min = i;
    for (int j = i + 1; j < n; j++)
    {
      if (arr[j] < arr[min])
      {
        min = j;
        temp = arr[i];
        arr[i] = arr[min];
        arr[min] = temp;
      }
    }
  }
  return arr[2];
}

void loop()
{
  Serial.println(code);
  switch (code)
  {
  case remote1:                        // Open non-continuous servo when button 1 is pressed
    followLine(motorLeft, motorRight); // follows the line using p control
    ultrasonicReading = ultrasonicReturn(rangefinder);
    Serial.println(ultrasonicReading); // since the robot is backwards, send in left and right flipped
    if (decoder.getKeyCode() == remoteStopMode)
    {
      motorLeft.setMotorEffort(0);
      motorRight.setMotorEffort(0);
      code = 1000;
    }
    else if (ultrasonicReading <= position2)
    {
      motorLeft.setMotorEffort(0);
      motorRight.setMotorEffort(0);
      motor.moveTo(fourtyFive);
      code = 1000;
    }
    break;
  case remote2:                        // Close non-continuous servo when button 2 is pressed
    followLine(motorLeft, motorRight); // follows the line using p control
    ultrasonicReading = ultrasonicReturn(rangefinder);
    Serial.println(ultrasonicReading); // since the robot is backwards, send in left and right flipped
    if (decoder.getKeyCode() == remoteStopMode)
    {
      motorLeft.setMotorEffort(0);
      motorRight.setMotorEffort(0);
      code = 1000;
    }
    else if (ultrasonicReading <= position2)
    {
      motorLeft.setMotorEffort(0);
      motorRight.setMotorEffort(0);
      motor.moveTo(twentyFive);
      code = 1000;
    }
    break;
  case remote3:
    followLine(motorLeft, motorRight); // follows the line using p control
    ultrasonicReading = ultrasonicReturn(rangefinder);
    Serial.println(ultrasonicReading);
    ; // since the robot is backwards, send in left and right flipped
    if (decoder.getKeyCode() == remoteStopMode)
    {
      motorLeft.setMotorEffort(0);
      motorRight.setMotorEffort(0);
      code = 1000;
    }
    else if (ultrasonicReading <= position1)
    {
      motorLeft.setMotorEffort(0);
      motorRight.setMotorEffort(0);
      code = 1000;
    }
    break;
  case remote4:
    motor.moveTo(clearance);
    code = 1000;
    break;
  case remote5:
    reverseDirection(750, motorLeft, motorRight);
    code = 1000;
    break;
  case remote6:
    followLine(motorLeft, motorRight);
    if (decoder.getKeyCode() == remoteStopMode)
    {
      motorLeft.setMotorEffort(0);
      motorRight.setMotorEffort(0);
      code = 1000;
    }
    else if (intersectionDetected)
    {
      motorLeft.setMotorEffort(0);
      motorRight.setMotorEffort(0);
      code = 1000;
    }
    break;
  case remote7:
    turnLeft(750, motorLeft, motorRight);
    if (decoder.getKeyCode() == remoteStopMode)
    {
      motorLeft.setMotorEffort(0);
      motorRight.setMotorEffort(0);
      code = 1000;
    }
    break;
  case remote8:
    turnRight(750, motorLeft, motorRight);
    if (decoder.getKeyCode() == remoteStopMode)
    {
      motorLeft.setMotorEffort(0);
      motorRight.setMotorEffort(0);
      code = 1000;
    }
    break;
  case remote9:
    followLine(motorLeft, motorRight); // follows the line using p control
    ultrasonicReading = ultrasonicReturn(rangefinder);
    Serial.println(ultrasonicReading); // since the robot is backwards, send in left and right flipped
    if (decoder.getKeyCode() == remoteStopMode)
    {
      motorLeft.setMotorEffort(0);
      motorRight.setMotorEffort(0);
      code = 1000;
    }
    else if (ultrasonicReading <= position4)
    {
      motorLeft.setMotorEffort(0);
      motorRight.setMotorEffort(0);
      motor.moveTo(flat);
      code = 1000;
    }
    break;
  case remoteUp:
    open(jawServo);
    if (decoder.getKeyCode() == remoteStopMode)
    {
      code = 1000;
    }
    code = 1000;
    break;
  case remoteDown:
    close(jawServo);
    if (decoder.getKeyCode() == remoteStopMode)
    {
      code = 1000;
    }
    code = 1000;
    break;
  case remoteLeft:
    motor.moveTo(flat);
    if (decoder.getKeyCode() == remoteStopMode)
    {
      code = 1000;
    }
    code = 1000;
    break;
  case remoteRight:
    motor.moveTo(clearance);
    if (decoder.getKeyCode() == remoteStopMode)
    {
      code = 1000;
    }
    code = 1000;
    break;
    case remoteEnterSave:
    followLine(motorLeft, motorRight); // follows the line using p control
    ultrasonicReading = ultrasonicReturn(rangefinder);
    Serial.println(ultrasonicReading); // since the robot is backwards, send in left and right flipped
    if (decoder.getKeyCode() == remoteStopMode)
    {
      motorLeft.setMotorEffort(0);
      motorRight.setMotorEffort(0);
      code = 1000;
    }
    else if (ultrasonicReading <= position1)
    {
      motorLeft.setMotorEffort(0);
      motorRight.setMotorEffort(0);
      code = 1000;
    }
    case remoteSetup:
    ultrasonicReading = ultrasonicReturn(rangefinder);
    Serial.println(ultrasonicReading);
    if (decoder.getKeyCode() == remoteStopMode)
    {
      motorLeft.setMotorEffort(0);
      motorRight.setMotorEffort(0);
      code = 1000;
    }
    break;
  default:
    code = decoder.getKeyCode();
  }
}
/*void loop()
{
  open(jawServo);
  delay(2000);
  close(jawServo);
  delay(2000);
  // Serial.println(analogRead(A0));
  // jawServo.writeMicroseconds(1000);
  // Serial.println(analogRead(A0));
  // delay(1000);
  // Serial.println(analogRead(A0));
  // //jawServo.writeMicroseconds(1250);
  // Serial.println(analogRead(A0));
  // delay(1000);
  // Serial.println(analogRead(A0));
}
/*
Serial.println(analogRead(A0));
jawServo.writeMicroseconds(1000);
Serial.println(analogRead(A0));
delay(1000);
Serial.println(analogRead(A0));
jawServo.writeMicroseconds(1250);
Serial.println(analogRead(A0));
delay(1000);
Serial.println(analogRead(A0));
// Serial.println(ultrasonicReturn(rangefinder));
/*Serial.print("Right Reading Is: ");
Serial.println(analogRead(20));
Serial.print("Left Reading Is: ");
Serial.println(analogRead(21));
delay(500);  */
// Serial.println(code);
/*switch (code)
{
case remote1:                        // Open non-continuous servo when button 1 is pressed
  followLine(motorLeft, motorRight); // since the robot is backwards, send in left and right flipped
  if (decoder.getKeyCode() == remote2)
  {
    code = remote2;
  }
  if (intersectionDetected(25))
  {
    motorLeft.setMotorEffort(0);
    motorRight.setMotorEffort(0);
    code = remote2;
    Serial.println("Place 4");
  }
  else
  {
    code = remote1;
  }
  Serial.println("Place 6");
  Serial.println(code);
  break;
case remote2: // Close non-continuous servo when button 2 is pressed
  Serial.println("Place 5");
  motorLeft.setMotorEffort(0);
  motorRight.setMotorEffort(0);
  code = 1000;
  break;
case remote3:
  motorLeft.setMotorEffort(0);
  motorRight.setMotorEffort(-100);
  break;
case remote4:
  moveUntilLine(motorLeft, motorRight);
  if (moveUntilLine(motorLeft, motorRight))
  {
    Serial.println("Here");
    motorLeft.setMotorEffort(0);
    motorRight.setMotorEffort(0);
    code = remote2;
  }
  if (decoder.getKeyCode() == remote2)
  {
    code = remote2;
  }
  break;
case remote5:
  turnLeft(750, motorLeft, motorRight);
  code = remote2;
  break;
case remote6:
  turnRight(750, motorLeft, motorRight);
  code = remote2;
  break;
case remote7:
  reverseDirection(750, motorLeft, motorRight);
  code = remote2;
  break;
case remote8:
  Serial.println(ultrasonicReturn(rangefinder));
  if (decoder.getKeyCode() == remote2)
  {
    code = remote2;
  }
  break;
case remoteStopMode: // Open non-continuous servo when button 1 is pressed
  motor.moveTo(flat);
  code = remote2;
  break;
case remotePlayPause: // Close non-continuous servo when button 2 is pressed
  motor.moveTo(twentyFive);
  code = remote2;
  break;
case remoteVolPlus: // Open continuous servo when button 3 is pressed
  motor.moveTo(fourtyFive);
  code = remote2;
  break;
case remoteSetup: // Close continuous servo when button 4 is pressed
  motor.moveTo(clearance);
  code = remote2;
  break;
case remoteUp: // Close continuous servo when button 4 is pressed
  motor.getCount();
  code = remote2;
  break;
case remoteLeft:
  open(jawServo);
  code = remote2;
  break;
case remoteRight:
  close(jawServo);
  code = remote2;
  break;
case remoteDown:
  Serial.println(analogRead(A0));
  if (decoder.getKeyCode() == remote2)
  {
    code = remote2;
  }
  break;
default:
  code = decoder.getKeyCode();
}
}*/

/*void loop(){
   int16_t code = decoder.getKeyCode();
   switch(code){
     case remote1: //Open non-continuous servo when button 1 is pressed
     motor.moveTo(flat);
     break;
     case remote2: //Close non-continuous servo when button 2 is pressed
     motor.moveTo(twentyFive);
     break;
     case remote3: //Open continuous servo when button 3 is pressed
     motor.moveTo(fourtyFive);
     break;
     case remote4: //Close continuous servo when button 4 is pressed
     motor.moveTo(clearance);
     break;
     case remote5: //Close continuous servo when button 4 is pressed
     open(jawServo);
     break;
     case remote6: //Close continuous servo when button 4 is pressed
     close(jawServo);
     break;
     }
   }*/

/*void loop(){
  open(jawServo);
  delay(2000);
  close(jawServo);
  delay(2000);
  //Serial.println(analogRead(A0));
  delay(2000);

  linPositionVoltADC = analogRead(linPositionVoltADC);
  Serial.print("Intitial linPosition Volt ADC is ");
  Serial.print(linPositionVoltADC);

  if(buttonB.isPressed()){ //opens and closes when button is pressed, always opens first
    change();

    if(mode == 1){
      open();
    }

    if(mode == 0){
      close();
    }
  }*/
//}

/*void loop() //use this code to test the dead band once the motor is in the gripper
{
  motor.deadBandTestNeg();
  motor.setEffort(0);
  Serial.println("0");
  delay(1000);
  // int value = ultrasonicReturn(rangefinder);
  // Serial.println(value);
  Serial.println(motor.getCount());
  delay(100);

}*/

// void loop() //this is what we ran when we had the move to function work perfect
// {
//   motor.moveTo(-540);

//   motor.setEffort(0);

// }

// void loop() //this one to show the quadrature stuff working
// {
//     Serial.println(motor.getCount());
// }

void updateStateMachine()
{
  switch (robotState)
  {
  case WAITING: // waits to start until button press
  {
    turncount = 0; // resets variables
    prevLocationWanted = 0;
    locationWanted = 2;
    openPosition = true;
    replacementComplete = false;
    Serial.println(A0);
    while (decoder.getKeyCode() == remote1)
    {                 // if 1 is pressed
      open(jawServo); // open jaw and go on path A
      robotState = PATHA;
    }
    while (decoder.getKeyCode() == remote2)
    {                 // if 2 is pressed
      open(jawServo); // open jaw and go on path B
      robotState = PATHB;
    }
    break;
  }

  case PATHA: // path A (right side, non-continuous)
  {
    switch (courseState)
    {
    case SERVO: // Handles the opening and closing of the jaw, know which to do based on a bool
      if (openPosition == true)
      {
        close(jawServo);
        openPosition = false;
      }
      else if (openPosition == false)
      {
        open(jawServo);
        openPosition = true;
      }
      if (prevLocationWanted == 4)
      { // if we go to servo when wanting to be in position 4, gp tp pausing
        courseState = PAUSING;
        break;
      }
      locationWanted = 2; // location wanted changes to 2 (line up position)
      courseState = BLUE_MOTOR;
      break;

    case BLUE_MOTOR: // controls the blue motor, based on where we are going and previous location at
      if (locationWanted == 2 && prevLocationWanted != 1)
      {                                      // if going to location 2 and was at location 4
        motor.moveTo(fourtyFive);            // raise motor to clearance
        prevLocationWanted = locationWanted; // update locations
        locationWanted = 1;
        courseState = LINE_FOLLOWING;
      }

      else if (locationWanted == 1)
      {                                      // if going to location 1
        prevLocationWanted = locationWanted; // update locations
        locationWanted = 2;
        motor.moveTo(clearance); // lower to twentyFive angle
        courseState = SERVO;
      }

      else if (locationWanted == 4)
      {                                      // if going to location 4
        prevLocationWanted = locationWanted; // update locations
        locationWanted = 2;
        motor.moveTo(flat); // lower to flat angle
        courseState = SERVO;
      }

      else
      {                                      // if at position 2 and was at location 1
        motor.moveTo(clearance);             // raise motor to clearance
        prevLocationWanted = locationWanted; // update locations
        locationWanted = 4;
        courseState = LINE_FOLLOWING;
      }
      break;

    case LINE_FOLLOWING: // follows lines and stops when at location or at intersection
      while (1 == 1)
      {
        followLine(motorLeft, motorRight); // follows the line using p control
        ultrasonicReading = ultrasonicReturn(rangefinder);
        Serial.println(ultrasonicReading);
        if (intersectionDetected(25))
        {                               // when an intersection is detected, romi stops, waits 300ms, then switches to turning
          motorRight.setMotorEffort(0); // the count of the intersections determines which way the romi is to turn, determined in turn()
          motorLeft.setMotorEffort(0);
          timeUpdateCheck(300);
          courseState = TURNING;
          break;
        }

        if (ultrasonicReading <= position1 && locationWanted == 1 && replacementComplete == false)
        {
          // stops when going to location 1 for replacement
          motorRight.setMotorEffort(0);
          motorLeft.setMotorEffort(0);
          courseState = SERVO;
          break;
        }

        if (ultrasonicReading <= position2 && locationWanted == 2 && replacementComplete == false)
        {
          // stops when going to location 2 for replacement
          motorRight.setMotorEffort(0);
          motorLeft.setMotorEffort(0);
          courseState = BLUE_MOTOR;
          break;
        }

        if (ultrasonicReading <= position4 && locationWanted == 4 && replacementComplete == false)
        {
          // stops when going to location 4 for replacement
          motorRight.setMotorEffort(0);
          motorLeft.setMotorEffort(0);
          courseState = BLUE_MOTOR;
          break;
        }

        if (ultrasonicReading <= position4 && locationWanted == 4 && replacementComplete == true)
        {
          // stops when going to location 4 for after replacement is complete
          motorRight.setMotorEffort(0);
          motorLeft.setMotorEffort(0);
          courseState = TURNING;
          break;
        }

        if (ultrasonicReading <= position7 && locationWanted == 7)
        {
          // stops when going to location 7 to be swapped out
          motorRight.setMotorEffort(0);
          motorLeft.setMotorEffort(0);
          robotState = WAITING;
          break;
        }
      }
      break;

    case TURNING:                                                // turns based on turncount and turnDirections
      turn(turncount, motorLeft, motorRight, turnDirectionsOne); // turns
      turncount++;
      if (turncount == 5)
      { // if turned 5 times, replacement is completed
        replacementComplete = true;
      }
      if (turncount == 7)
      { // after turning 7 times, go to the other side
        locationWanted = 7;
      }
      break;

    case PAUSING: // waiting for input
      if (decoder.getKeyCode() == remotePlayPause)
      { // when button is pressed, close servo and begin moving
        close(jawServo);
        openPosition = false;
        locationWanted = 2;
        courseState = BLUE_MOTOR;
      }
      break;

    case MOVE_TO_LINE: // move until a line is found
      moveUntilLine(motorLeft, motorRight);
      courseState = TURNING;
      break;
    }
    break;
  }

  case PATHB: // very similar to PATHA, just using continuous servo and inverted turns
  {
    switch (courseState)
    {
    case SERVO:
      if (openPosition == true)
      {
        close(jawServo);
        openPosition = false;
      }
      else if (openPosition == false)
      {
        open(jawServo);
        openPosition = true;
      }
      if (prevLocationWanted == 4)
      {
        courseState = PAUSING;
        break;
      }
      locationWanted = 2;
      courseState = BLUE_MOTOR;
      break;

    case BLUE_MOTOR:
      if (locationWanted == 2 && prevLocationWanted != 1)
      {
        motor.moveTo(clearance);
        prevLocationWanted = locationWanted;
        locationWanted = 1;
        courseState = LINE_FOLLOWING;
      }

      else if (locationWanted == 1)
      {
        prevLocationWanted = locationWanted;
        locationWanted = 2;
        motor.moveTo(twentyFive);
        courseState = SERVO;
      }

      else if (locationWanted == 4)
      {
        prevLocationWanted = locationWanted;
        locationWanted = 2;
        motor.moveTo(flat);
        courseState = SERVO;
      }

      else
      {
        motor.moveTo(clearance);
        prevLocationWanted = locationWanted;
        locationWanted = 4;
        courseState = LINE_FOLLOWING;
      }
      break;

    case LINE_FOLLOWING:
      followLine(motorLeft, motorRight); // follows the line using p control
      ultrasonicReading = ultrasonicReturn(rangefinder);
      if (intersectionDetected(25))
      {                               // when an intersection is detected, romi stops, waits 300ms, then switches to turning
        motorRight.setMotorEffort(0); // the count of the intersections determines which way the romi is to turn, determined in turn()
        motorLeft.setMotorEffort(0);
        timeUpdateCheck(300);
        courseState = TURNING;
        break;
      }

      if (ultrasonicReading == position1 && locationWanted == 1 && replacementComplete == false)
      {
        courseState = BLUE_MOTOR;
        break;
      }

      if (ultrasonicReading == position2 && locationWanted == 2 && replacementComplete == false)
      {
        courseState = BLUE_MOTOR;
        break;
      }

      if (ultrasonicReading == position4 && locationWanted == 4 && replacementComplete == false)
      {
        courseState = BLUE_MOTOR;
        break;
      }

      if (ultrasonicReading == position4 && locationWanted == 4 && replacementComplete == true)
      {
        courseState = TURNING;
        break;
      }

      if (ultrasonicReading == position7 && locationWanted == 7)
      {
        robotState = WAITING;
        break;
      }
      break;

    case TURNING:
      turn(turncount, motorLeft, motorRight, turnDirectionsTwo);
      turncount++;
      if (turncount == 5)
      {
        replacementComplete = true;
      }
      if (turncount == 7)
      {
        locationWanted = 7;
      }
      break;

    case PAUSING:
      if (decoder.getKeyCode() == remotePlayPause)
      {
        close(jawServo);
        openPosition = false;
        locationWanted = 2;
        courseState = BLUE_MOTOR;
      }
      break;

    case MOVE_TO_LINE:
      moveUntilLine(motorLeft, motorRight);
      courseState = TURNING;
      break;
    }
    break;
  }
  }
}

/*void loop()
{
  updateStateMachine();
}*/