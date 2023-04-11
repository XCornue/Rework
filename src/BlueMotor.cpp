#include <Arduino.h>
#include <BlueMotor.h>
#include <Romi32U4.h>
#include <math.h>

long oldValue = 0;
long newValue;
long count = 0;
unsigned time = 0;
long kp = 1;
int posDeadBand = 300;  //best experimental value == 75
int negDeadBand = -169;
long mPos = (400-posDeadBand)/400;
long mNeg = (400-negDeadBand)/400;

BlueMotor::BlueMotor()
{
}

void BlueMotor::setup()
{
    pinMode(PWMOutPin, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    TCCR1A = 0xA8; //0b10101000; //gcl: added OCR1C for adding a third PWM on pin 11
    TCCR1B = 0x11; //0b00010001;
    ICR1 = 400;
    OCR1C = 0;

    attachInterrupt(digitalPinToInterrupt(ENCA), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB), isrB, CHANGE);
    reset();
}

long BlueMotor::getPosition()
{
    long tempCount = 0;
    noInterrupts();
    tempCount = count;
    interrupts();
    return tempCount;
}

long BlueMotor::getCount(){
    return count;
}

void BlueMotor::reset()
{
    noInterrupts();
    count = 0;
    interrupts();
}


void BlueMotor::isrA()
{
    if(digitalRead(ENCA) == digitalRead(ENCB)){
        count++;
    } else {
        count--;
    }
}

void BlueMotor::isrB()
{
    if(digitalRead(ENCA) != digitalRead(ENCB)){
        count++;
    } else {
        count--;
    }
}

void BlueMotor::setEffort(int effort)
{
    if (effort < 0)
    {
        setEffort(-effort, false);
    }
    else
    {
        setEffort(effort, true);
    }
}

void BlueMotor::setEffortWithoutDB(int effort) // accepts range from -400 to 400, 
                                               //needs to translate the input into the actual effort in the other function

{
    long eff; //temp variable
    if (effort > 0) //if the effort is positive
    {
        eff = round(mPos*effort + posDeadBand); //adjust to get rid of deadzone
        setEffort((int) eff, true); //set the effort to the new value
    }
    else if (effort <0) //if the effort is negative
    {
        eff = round(mNeg*effort + negDeadBand); //adjust to get rid of deadzone
        setEffort((int) -eff, false); //set the effort to the new value
    } else 
    {
        setEffort(0); //set effort to zero
    }
}

void BlueMotor::setEffort(int effort, bool clockwise)
{
    if (clockwise) //set motor to spin clockwise
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else //set motor to spin counterclockwise
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    OCR1C = constrain(effort, 0, 400); //make the motor move at certain speed
}

void BlueMotor::deadBandTestPos() //attach blue motor to assembly for this test
{
    count = 0;
    for(int i = 0; i < 399; i++){
        setEffort(i);
        Serial.print(i);
        Serial.print("  ");
        Serial.println(count);
        delay(100);
    }
}

void BlueMotor::deadBandTestNeg() //attach blue motor to assembly for this test
{
    count = 0;
    for(int i = 0; i > -399; i--){
        setEffort(i);
        Serial.print(i);
        Serial.print("  ");
        Serial.println(count);
        delay(100);
    }
}


void BlueMotor::moveTo(long target)  //Move to this encoder position within the specified
{                                    //tolerance in the header file using proportional control
                                     //then stop 
                                     //note 540 counts per rev
                                     //tolerance of 3 counts
    long eff;
    while((target-count)>= tolerance || (target-count)<= (-tolerance)){ //while the target is not within the tolerance
        eff = kp*(target - count); //the effort equals the target - count adjusted by a proportional value
        setEffortWithoutDB((int) eff); //send the effort to the without deadband 
        Serial.println("in loop"); //for testing
        Serial.print(count); //for testing
    } 
    setEffort(0); //stop the motor
    Serial.println("out of loop"); //for testing
    Serial.print(count); //for testing
}

// // //perfect move to function (in our experiments)
// void BlueMotor::moveTo(long target)  //Move to this encoder position within the specified
// {                                    //tolerance in the header file using proportional control
//                                      //then stop 
//                                      //note 540 counts per rev
//                                      //tolerance of 3 counts
//     long eff;
//     while((target-count)>= tolerance || (target-count)<= (-tolerance)){
//         eff = kp*(target - count);
//         if(eff >= 0){
//             if(eff < deadBand){
//                 eff = deadBand;
//             }
//             setEffort(eff,true);
//         } else{
//             if(eff > -deadBand){
//                 eff = -deadBand;
//             }
//             setEffort(abs(eff),false);
//         }
//         Serial.println("in loop");
//         Serial.print(count);
//     }
//     setEffort(0);
//     Serial.println("out of loop");
//     Serial.print(count);
// }