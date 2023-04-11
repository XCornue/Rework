#include "Romi32U4Motors.h"

bool moveUntilLine(LeftMotor motorLeft, RightMotor motorRight);
bool lineDetectedRight();
bool lineDetectedLeft();

bool timeUpdateCheck(int samplePeriod);

void turn(int turncount, LeftMotor motorLeft, RightMotor motorRight, int instructions[9]);
void turnRightReverse(int samplePeriod, LeftMotor motorLeft, RightMotor motorRight);
void turnLeftReverse(int samplePeriod, LeftMotor motorLeft, RightMotor motorRight);
void turnLeft(int samplePeriod, LeftMotor motorLeft, RightMotor motorRight);
void turnRight(int samplePeriod, LeftMotor motorLeft, RightMotor motorRight);
void reverseDirection(int samplePeriod, LeftMotor motorLeft, RightMotor motorRight);

bool turnLeftHelper();
bool turnRightHelper(int samplePeriod);
bool reverseDirectionHelper(int samplePeriod);

bool intersectionDetected(int samplePeriod);

float piControl(int target, int actual, float Kp, float Ki);
void followLine(LeftMotor motorLeft, RightMotor motorRight);