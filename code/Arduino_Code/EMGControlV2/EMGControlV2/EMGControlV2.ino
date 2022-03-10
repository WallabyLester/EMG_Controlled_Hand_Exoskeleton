/********************************************************************
 * Read analog input of Myoware EMG sensor and convert it to voltage 
 * to control motor. 
 ********************************************************************/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

int sensor1Pin = A0;
int sensor2Pin = A1;
int sensor3Pin = A2;
int sensor4Pin = A3;
int freq = 1000;    
int sensorVal1 = 0;
int sensorVal2 = 0;
int sensorVal3 = 0;
int sensorVal4 = 0;
// String sensorLabel1 = "EMG Sensor 1";
bool flag = true;
int state = 0;


void setup() 
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  AFMS.begin();
}

void loop() 
{
  sensorVal1 = analogRead(sensor1Pin);            // read the input on analog pin 0
  float voltage1 = sensorVal1 * (5.0 / 1023.0);   // Convert the analog reading to a voltage

  sensorVal2 = analogRead(sensor2Pin);           
  float voltage2 = sensorVal2 * (5.0 / 1023.0);

  sensorVal3 = analogRead(sensor3Pin);           
  float voltage3 = sensorVal3 * (5.0 / 1023.0);

  sensorVal4 = analogRead(sensor4Pin);           
  float voltage4 = sensorVal4 * (5.0 / 1023.0); 

  // stop movement
  if (voltage1 < 0.5)
  {
      myMotor1->run(RELEASE);
      myMotor2->run(RELEASE);
      myMotor3->run(RELEASE);
      myMotor4->run(RELEASE);
      state = 0;
  }

  // move index finger
  if (voltage1 >= 0.8 && voltage1 < 1.2 && voltage2 >= 0.09 && voltage3 >=0.2 && voltage3 < 0.55 && voltage4 >= 0.4 && voltage4 < 0.6)
  {
      state = 1;
      run(myMotor1);
  }

  // move middle finger
  if (voltage1 >= 1.0 && voltage1 < 1.8 && voltage2 >= 0.1 && voltage2 <= 0.25 && voltage3 >= 0.6 && voltage3 < 1.5 && voltage4 >= 1.25 && voltage4 < 2.0)
  {
      state = 1;
      run(myMotor2);
  }

  // move fourth finger
  if (voltage1 >= 1.2 && voltage1 < 2.0 && voltage2 >= 0.17 && voltage2 <= 0.5 && voltage3 >= 0.5 && voltage3 < 0.9 && voltage4 >= 1.0 && voltage4 < 1.3)
  {
      state = 1;
      run(myMotor3);
  }

  // move pinky finger
  if (voltage1 >= 0.5 && voltage1 < 0.7 && voltage2 >= 0.1 && voltage2 <= 0.2 && voltage3 >= 0.2 && voltage3 < 0.65 && voltage4 >= 1.0 && voltage4 < 1.2)
  {
      state = 1;
      run(myMotor4);
  }
}

void run (Adafruit_DCMotor* motor)
{
    while (state == 1)
    {
      motor->setSpeed(100);
      motor->run(FORWARD);
      delay(freq);
      motor->run(RELEASE);
      delay(freq);
      motor->run(BACKWARD);
      delay(freq);
      motor->run(RELEASE);
      delay(freq);
      state = 0;
    }
}
