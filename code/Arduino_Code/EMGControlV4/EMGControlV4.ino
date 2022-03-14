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

#define ENCA 2  
#define ENCB 3  

int sensor1Pin = A0;
int sensor2Pin = A1;
int sensor3Pin = A2;
int sensor4Pin = A3;
int freq = 1000;    
int sensorVal1 = 0;
int sensorVal2 = 0;
int sensorVal3 = 0;
int sensorVal4 = 0;
int state = 0;
int incomingByte = 0;
int pos = 0, target = 0, dir = 0;
long prevT = 0;
float eprev = 0, eintegral = 0, pwr = 0;

void bend(Adafruit_DCMotor* motor, int _dir, float power);
void stop();
void readEncoder();
void PID(int goal);

void setup() 
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  AFMS.begin();
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
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
      // go to starting position
      PID(0);
      bend(myMotor1, dir, pwr);
    }
    
    if (incomingByte == '1') {
      state = 1;
      // move index finger 
      PID(-100);
      bend(myMotor1, dir, pwr);
    }

    if (incomingByte == '2') {
      state = 2;
      // move middle finger
      PID(-100);
      bend(myMotor2, dir, pwr);
    }

    if (incomingByte == '3') {
      state = 3;
      // move fourth finger
      PID(-100);
      bend(myMotor3, dir, pwr);
    }

    if (incomingByte == '4') {
      state = 4;
      // move pinky finger
      PID(-100);
      bend(myMotor4, dir, pwr);
    }
  }
}

void bend(Adafruit_DCMotor* motor, int _dir, float power)
{
    motor->setSpeed(power);

    if (_dir == 1)
    {
        motor->run(FORWARD);
        delay(freq);
        motor->run(RELEASE);
        delay(freq);
    }
    else if (_dir == -1)
    {
        motor->run(BACKWARD);
        delay(freq);
        motor->run(RELEASE);
        delay(freq);
    }
    else
    {
        myMotor1->run(RELEASE);
        myMotor2->run(RELEASE);
        myMotor3->run(RELEASE);
        myMotor4->run(RELEASE);
    }
}

void stop()
{
    myMotor1->run(RELEASE);
    myMotor2->run(RELEASE);
    myMotor3->run(RELEASE);
    myMotor4->run(RELEASE);
}

void PID(int goal)
{
    // PID constants
    float kp = 0.1;
    float ki = 0;
    float kd = 0.1;

    // time difference 
    long currT = micros();

    float deltaT = ((float)(currT-prevT))/1.0e6;
    prevT = currT;

    // error
    int e = pos-goal; 

    // derivative 
    float dedt = (e-eprev)/(deltaT);

    // integral 
    eintegral = eintegral + e*deltaT;

    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;

    pwr = fabs(u);

    // motor direction 
    dir = 1;

    if(u<0)
    {
        dir = -1;
    }

    // store previous error
    eprev = e;
}

void readEncoder()
{
    int b = digitalRead(ENCB);
    if(b>0)
    {
        pos++;  //CW (up)
    }
    else
    {
        pos--;  //CCW (down)
    }
}