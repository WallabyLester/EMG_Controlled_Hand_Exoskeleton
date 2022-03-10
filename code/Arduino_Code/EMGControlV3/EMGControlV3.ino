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
bool flag = true;
int state = 0;
int incomingByte = 0;

void bend(Adafruit_DCMotor* motor);
void home();

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

  Serial.print(voltage1);
  Serial.print(",");
  Serial.print(voltage2);
  Serial.print(",");
  Serial.print(voltage3);
  Serial.print(",");
  Serial.println(voltage4);

  delay(freq); 

  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();

    if (incomingByte == '0') {
      state = 0;
      // stop movement
      home();
    }
    
    if (incomingByte == '1') {
      state = 1;
      // move index finger 
      bend(myMotor1);
    }

    if (incomingByte == '2') {
      state = 2;
      // move middle finger
      bend(myMotor2);
    }

    if (incomingByte == '3') {
      state = 3;
      // move fourth finger
      bend(myMotor3);
    }

    if (incomingByte == '4') {
      state = 4;
      // move pinky finger
      bend(myMotor4);
    }
  }
}

void bend(Adafruit_DCMotor* motor)
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
}

void home()
{
    myMotor1->run(RELEASE);
    myMotor2->run(RELEASE);
    myMotor3->run(RELEASE);
    myMotor4->run(RELEASE);
}
