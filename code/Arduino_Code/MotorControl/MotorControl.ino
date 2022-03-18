/********************************************************************
 * Code to test motor movement individually.
*********************************************************************/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Initialize motor driver
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

int freq = 1000;

void setup() 
{
    AFMS.begin();
}

void loop() 
{
    while(1)
    {
        // Move motor forward and backward
        myMotor->setSpeed(70);
        myMotor->run(FORWARD);
        delay(freq);
        myMotor->run(RELEASE);
        delay(freq);
        myMotor->run(BACKWARD);
        delay(freq);
        myMotor->run(RELEASE);
        delay(freq);
    }
}
