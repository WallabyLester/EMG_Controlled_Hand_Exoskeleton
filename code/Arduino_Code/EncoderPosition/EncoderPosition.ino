#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

#define ENCA_index 2  // purple
#define ENCB_index 3  // blue
#define ENCA_middle 4  // purple
#define ENCB_middle 5  // blue
#define ENCA_fourth 7  // purple
#define ENCB_fourth 6  // blue
#define ENCA_pinky 8  // purple
#define ENCB_pinky 9  // blue

int pos_index = 0;
int pos_middle = 0;
int pos_fourth = 0;
int pos_pinky = 0; 

int a_middle_prev = 0;
int a_middle = 0;
int b_middle_prev = 0;
int b_middle = 0;

int a_fourth_prev = 0;
int a_fourth = 0;
int b_fourth_prev = 0;
int b_fourth = 0;

int a_pinky_prev = 0;
int a_pinky = 0;
int b_pinky_prev = 0;
int b_pinky = 0;

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

    pinMode(ENCA_index, INPUT);
    pinMode(ENCB_index, INPUT);
    pinMode(ENCA_middle, INPUT);
    pinMode(ENCB_middle, INPUT);
    pinMode(ENCA_fourth, INPUT);
    pinMode(ENCB_fourth, INPUT);
    pinMode(ENCA_pinky, INPUT);
    pinMode(ENCB_pinky, INPUT);

    cli();
    TCCR2A = 0b01010011;
    TCCR2B = 0b00001100;
    OCR2A = 84;
    OCR2B = OCR2A/2; 
    TIMSK1 |= B00000010;
    sei();

    attachInterrupt(digitalPinToInterrupt(ENCA_index), readIndexEncoder, RISING);
}

void loop()
{
    a_middle_prev = digitalRead(ENCA_middle);
    b_middle_prev = digitalRead(ENCB_middle);

    a_fourth_prev = digitalRead(ENCA_fourth);
    b_fourth_prev = digitalRead(ENCA_fourth);

    a_pinky_prev = digitalRead(ENCA_pinky);
    b_pinky_prev = digitalRead(ENCA_pinky);

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
    int e = pos_index-target; // can switch order depending on motor leads

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

void readIndexEncoder()
{
    int b_index = digitalRead(ENCB_index);
    if(b_index>0)
    {
        pos_index++;  //CW (up)
    }
    else
    {
        pos_index--;  //CCW (down)
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
        Serial.print(pos_index);
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
        Serial.print(pos_index);
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

ISR(TIMER1_COMPA_vect)    
{  
    // middle finger
    a_middle = digitalRead(ENCA_middle);

    // RISING: from LOW to HIGH
    if (a_middle_prev == LOW && a_middle == HIGH)
    {
      b_middle = digitalRead(ENCB_middle);

      if(a_middle_prev == LOW && b_middle_prev == LOW && a_middle == LOW && b_middle == HIGH)
      {
          pos_middle--; 
      }
      else if (a_middle_prev == LOW && b_middle_prev == LOW && a_middle == HIGH && b_middle == LOW)
      {
          pos_middle++;
      }
      else if (a_middle_prev == LOW && b_middle_prev == HIGH && a_middle == HIGH && b_middle == HIGH)
      {
          pos_middle--; 
      }
      else if (a_middle_prev == LOW && b_middle_prev == HIGH && a_middle == LOW && b_middle == LOW)
      {
          pos_middle++; 
      }
      else if (a_middle_prev == HIGH && b_middle_prev == LOW && a_middle == LOW && b_middle == LOW)
      {
          pos_middle--; 
      }
      else if (a_middle_prev == HIGH && b_middle_prev == LOW && a_middle == HIGH && b_middle == HIGH)
      {
          pos_middle++; 
      }
      else if (a_middle_prev == HIGH && b_middle_prev == HIGH && a_middle == HIGH && b_middle == LOW)
      {
          pos_middle--; 
      }
      else if (a_middle_prev == HIGH && b_middle_prev == HIGH && a_middle == LOW && b_middle == HIGH)
      {
          pos_middle++; 
      }
    }
    a_middle_prev = a_middle;
    b_middle_prev = b_middle;

    // fourth finger
    a_fourth = digitalRead(ENCA_fourth);

    // RISING: from LOW to HIGH
    if (a_fourth_prev == LOW && a_fourth == HIGH)
    {
      b_fourth = digitalRead(ENCB_fourth);

      if(a_fourth_prev == LOW && b_middle_prev == LOW && a_fourth == LOW && b_fourth == HIGH)
      {
          pos_fourth--; 
      }
      else if (a_fourth_prev == LOW && b_middle_prev == LOW && a_fourth == HIGH && b_fourth == LOW)
      {
          pos_fourth++;
      }
      else if (a_fourth_prev == LOW && b_middle_prev == HIGH && a_fourth == HIGH && b_fourth == HIGH)
      {
          pos_fourth--; 
      }
      else if (a_fourth_prev == LOW && b_middle_prev == HIGH && a_fourth == LOW && b_fourth == LOW)
      {
          pos_fourth++; 
      }
      else if (a_fourth_prev == HIGH && b_middle_prev == LOW && a_fourth == LOW && b_fourth == LOW)
      {
          pos_fourth--; 
      }
      else if (a_fourth_prev == HIGH && b_middle_prev == LOW && a_fourth == HIGH && b_fourth == HIGH)
      {
          pos_fourth++; 
      }
      else if (a_fourth_prev == HIGH && b_middle_prev == HIGH && a_fourth == HIGH && b_fourth == LOW)
      {
          pos_fourth--; 
      }
      else if (a_fourth_prev == HIGH && b_middle_prev == HIGH && a_fourth == LOW && b_fourth == HIGH)
      {
          pos_fourth++; 
      }
    }
    a_fourth_prev = a_fourth;
    b_fourth_prev = b_fourth;

    // pinky
    a_pinky = digitalRead(ENCA_pinky);

    // RISING: from LOW to HIGH
    if (a_pinky_prev == LOW && a_pinky == HIGH)
    {
      b_pinky = digitalRead(ENCB_pinky);

      if(a_pinky_prev == LOW && b_middle_prev == LOW && a_pinky == LOW && b_pinky == HIGH)
      {
          pos_pinky--; 
      }
      else if (a_pinky_prev == LOW && b_middle_prev == LOW && a_pinky == HIGH && b_pinky == LOW)
      {
          pos_pinky++;
      }
      else if (a_pinky_prev == LOW && b_middle_prev == HIGH && a_pinky == HIGH && b_pinky == HIGH)
      {
          pos_pinky--; 
      }
      else if (a_pinky_prev == LOW && b_middle_prev == HIGH && a_pinky == LOW && b_pinky == LOW)
      {
          pos_pinky++; 
      }
      else if (a_pinky_prev == HIGH && b_middle_prev == LOW && a_pinky == LOW && b_pinky == LOW)
      {
          pos_pinky--; 
      }
      else if (a_pinky_prev == HIGH && b_middle_prev == LOW && a_pinky == HIGH && b_pinky == HIGH)
      {
          pos_pinky++; 
      }
      else if (a_pinky_prev == HIGH && b_middle_prev == HIGH && a_pinky == HIGH && b_pinky == LOW)
      {
          pos_pinky--; 
      }
      else if (a_pinky_prev == HIGH && b_middle_prev == HIGH && a_pinky == LOW && b_pinky == HIGH)
      {
          pos_pinky++; 
      }
    }
    a_pinky_prev = a_pinky;
    b_pinky_prev = b_pinky;
}

