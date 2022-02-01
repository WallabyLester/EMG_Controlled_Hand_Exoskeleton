/********************************************************************
 * Read analog input of Myoware EMG sensor and convert it to voltage 
 * to control motor. 
 ********************************************************************/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

int sensor1Pin = A0;
int freq = 1000;    //data collection frequency
int sensorVal = 0;
String sensorLabel1 = "EMG Sensor 1";
bool flag = true;
int state = 0;


void setup() 
{
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  AFMS.begin();
}

void loop() 
{
  sensorVal = analogRead(sensor1Pin);           // read the input on analog pin 0
  float voltage = sensorVal * (5.0 / 1023.0);   // Convert the analog reading to a voltage
  if (voltage < 0.5)
  {
      myMotor->run(RELEASE);
      state = 0;
  }
  if (voltage > 0.5)
  {
      state = 1;
      run();
  }
}


void run ()
{
    while (state == 1)
    {
      myMotor->setSpeed(100);
      myMotor->run(FORWARD);
      delay(freq);
      myMotor->run(RELEASE);
      delay(freq);
      myMotor->run(BACKWARD);
      delay(freq);
      myMotor->run(RELEASE);
      delay(freq);
      state = 0;
    }
}
