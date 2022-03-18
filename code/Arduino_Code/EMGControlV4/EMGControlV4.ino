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

int sensor1Pin = A0;
int sensor2Pin = A1;
int sensor3Pin = A2;
int sensor4Pin = A3;

int sensorVal1 = 0;
int sensorVal2 = 0;
int sensorVal3 = 0;
int sensorVal4 = 0;

int freq = 1000; 
int state = 0;
int incomingByte = 0;
int target = 0, dir = 0;
long prevT = 0;
float eprev = 0, eintegral = 0, pwr = 0;

void bend(Adafruit_DCMotor* motor, int _dir, float power);
void stop();
void PID(int pos, int goal);

void setup() 
{
  // initialize serial communication at 9600 bits per second:
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

  a_middle_prev = digitalRead(ENCA_middle);
  b_middle_prev = digitalRead(ENCB_middle);

  a_fourth_prev = digitalRead(ENCA_fourth);
  b_fourth_prev = digitalRead(ENCA_fourth);

  a_pinky_prev = digitalRead(ENCA_pinky);
  b_pinky_prev = digitalRead(ENCA_pinky);

  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();

    if (incomingByte == '0') {
      state = 0;
      // go to starting position
      PID(pos_index, 0);
//      bend(myMotor1, dir, pwr);
//      PID(pos_middle, 0);
//      bend(myMotor2, dir, pwr);
//      PID(pos_fourth, 0);
//      bend(myMotor3, dir, pwr);
//      PID(pos_pinky, 0);
//      bend(myMotor4, dir, pwr);
    }
    
    if (incomingByte == '1') {
      state = 1;
      // move index finger 
      PID(pos_index, -500);
      bend(myMotor1, dir, pwr);
      PID(pos_index, 200);
      bend(myMotor1, dir, pwr);
    }

    if (incomingByte == '2') {
      state = 2;
      // move middle finger
      PID(pos_middle, -400);
      bend(myMotor2, dir, pwr);
      PID(pos_middle, 200);
      bend(myMotor2, dir, pwr);
    }

    if (incomingByte == '3') {
      state = 3;
      // move fourth finger
      PID(pos_fourth, -400);
      bend(myMotor3, dir, pwr);
      PID(pos_fourth, 200);
      bend(myMotor3, dir, pwr);
    }

    if (incomingByte == '4') {
      state = 4;
      // move pinky finger
      PID(pos_pinky, -400);
      bend(myMotor4, dir, pwr);
      PID(pos_pinky, 200);
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

void PID(int pos, int goal)
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
