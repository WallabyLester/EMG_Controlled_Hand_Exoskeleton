/********************************************************************
 * Code to stop motor movement. 
*********************************************************************/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Initialize motor driver
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

int freq = 1000;

void setup() 
{
    AFMS.begin();
}

void loop() 
{
    while(1)
    {
      myMotor1->run(RELEASE);
      myMotor2->run(RELEASE);
      myMotor3->run(RELEASE);
      myMotor4->run(RELEASE);
    }
}
