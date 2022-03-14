#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

#define ENCA 2  // yellow
#define ENCB 3  // white

int pos = 0;
int freq = 1000;
int state = 0;
int target = 0;
int dir = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void bend(Adafruit_DCMotor* motor, int dirs, float power);
void home();
void readEncoder();

void setup()
{
    Serial.begin(9600);
    AFMS.begin();
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop()
{
    // set target position
    target = -100;

    // PID constants
    float kp = 0.1;
    float ki = 0;
    float kd = 0.1;

    // time difference 
    long currT = micros();

    float deltaT = ((float)(currT-prevT))/1.0e6;
    prevT = currT;

    // error
    int e = pos-target; // can switch order depending on motor leads

    // derivative 
    float dedt = (e-eprev)/(deltaT);

    // integral 
    eintegral = eintegral + e*deltaT;

    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;

    float pwr = fabs(u);
//    if(pwr>255)
//    {
//        pwr = 255;
//    }

    // motor direction 
    dir = 1;
    if(u<0)
    {
        dir = -1;
    }
    
    // move index finger 
    bend(myMotor1, dir, pwr);

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

void bend(Adafruit_DCMotor* motor, int dirs, float power)
{
    motor->setSpeed(power);
    if (dirs == 1)
    {
        motor->run(FORWARD);
        delay(freq);
        Serial.print(power);
        Serial.print(" ");
        Serial.print(target);
        Serial.print(" ");
        Serial.print(pos);
        Serial.println();
        motor->run(RELEASE);
        delay(freq);
    }
    else if (dirs == -1)
    {
        motor->run(BACKWARD);
        delay(freq);
        Serial.print(power);
        Serial.print(" ");
        Serial.print(target);
        Serial.print(" ");
        Serial.print(pos);
        Serial.println();
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
