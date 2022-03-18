/********************************************************************
 * Read analog input of Myoware EMG sensor and convert it to voltage 
 * to control motor using encoders.
 * 
 * Implements external and internal interrupts to read encoders PID
 * feedback control loop.
 ********************************************************************/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Initialize motor driver and each motor
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

// Define encoder channel pins
#define ENCA_index 2    // Only pins 2 and 3 are interrupt pins on Arduino Uno
#define ENCB_index 3
#define ENCA_middle 4  
#define ENCB_middle 5
#define ENCA_fourth 7  
#define ENCB_fourth 6
#define ENCA_pinky 8  
#define ENCB_pinky 9 

// Initialize encoder position counters
int pos_index = 0;
int pos_middle = 0;
int pos_fourth = 0;
int pos_pinky = 0; 

// Initialize encoder channel readings
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

// Define sensor pins and initialize values
int sensor1Pin = A0;
int sensor2Pin = A1;
int sensor3Pin = A2;
int sensor4Pin = A3;

int sensorVal1 = 0;
int sensorVal2 = 0;
int sensorVal3 = 0;
int sensorVal4 = 0;

int freq = 1000; 
int incomingByte = 0;

// Constants for feedback control loop
int target = 0, dir = 0;
long prevT = 0;
float eprev = 0, eintegral = 0, pwr = 0;

// Function prototypes
void bend(Adafruit_DCMotor* motor, int _dir, float power);
void stop();
void PID(int pos, int goal);

void setup() 
{
    // Initialize serial communication at 9600 bits per second
    Serial.begin(9600);
    AFMS.begin();

    // Set encoder pins as inputs to read channels
    pinMode(ENCA_index, INPUT);
    pinMode(ENCB_index, INPUT);
    pinMode(ENCA_middle, INPUT);
    pinMode(ENCB_middle, INPUT);
    pinMode(ENCA_fourth, INPUT);
    pinMode(ENCB_fourth, INPUT);
    pinMode(ENCA_pinky, INPUT);
    pinMode(ENCB_pinky, INPUT);

    // Initialize Arduino Timer 1: 16 bit timer
    cli();
    TCCR2A = 0b01010011;
    TCCR2B = 0b00001100;
    OCR2A = 84;
    OCR2B = OCR2A/2; 
    TIMSK1 |= B00000010;
    sei();

    // Using built in interrupt pins for index finger
    attachInterrupt(digitalPinToInterrupt(ENCA_index), readIndexEncoder, RISING);
}

void loop() 
{
    sensorVal1 = analogRead(sensor1Pin);            // read the input on analog pin
    float voltage1 = sensorVal1 * (5.0 / 1023.0);   // Convert the analog reading to a voltage

    sensorVal2 = analogRead(sensor2Pin);           
    float voltage2 = sensorVal2 * (5.0 / 1023.0);

    sensorVal3 = analogRead(sensor3Pin);           
    float voltage3 = sensorVal3 * (5.0 / 1023.0);

    sensorVal4 = analogRead(sensor4Pin);           
    float voltage4 = sensorVal4 * (5.0 / 1023.0); 

    // Output readings in serial buffer
    Serial.print(voltage1);
    Serial.print(",");
    Serial.print(voltage2);
    Serial.print(",");
    Serial.print(voltage3);
    Serial.print(",");
    Serial.println(voltage4);

    delay(freq); 

    // Read encoder channels for current state
    a_middle_prev = digitalRead(ENCA_middle);
    b_middle_prev = digitalRead(ENCB_middle);

    a_fourth_prev = digitalRead(ENCA_fourth);
    b_fourth_prev = digitalRead(ENCA_fourth);

    a_pinky_prev = digitalRead(ENCA_pinky);
    b_pinky_prev = digitalRead(ENCA_pinky);

    if (Serial.available() > 0) 
    {
        // read the oldest byte in the serial buffer
        incomingByte = Serial.read();

        if (incomingByte == '0') 
        {
            // Classified as no flexion, will return fingers to starting positions
            // Works ideally with index finger; however, encoder resolution is an
            // issue with timer run encoders
            // PID(pos_index, 0);
            // bend(myMotor1, dir, pwr);
            // PID(pos_middle, 0);
            // bend(myMotor2, dir, pwr);
            // PID(pos_fourth, 0);
            // bend(myMotor3, dir, pwr);
            // PID(pos_pinky, 0);
            // bend(myMotor4, dir, pwr);
        }
    
        if (incomingByte == '1') 
        {
            // Classified as index finger, sends desired encoder positions to press down and lift
            PID(pos_index, -500);
            bend(myMotor1, dir, pwr);
            PID(pos_index, 200);
            bend(myMotor1, dir, pwr);
        }

        if (incomingByte == '2') 
        {
            // Classified as middle finger
            PID(pos_middle, -400);
            bend(myMotor2, dir, pwr);
            PID(pos_middle, 200);
            bend(myMotor2, dir, pwr);
        }

        if (incomingByte == '3') 
        {
            // Classified as fourth finger
            PID(pos_fourth, -400);
            bend(myMotor3, dir, pwr);
            PID(pos_fourth, 200);
            bend(myMotor3, dir, pwr);
        }

        if (incomingByte == '4') 
        {
            // Classified as pinky finger
            PID(pos_pinky, -400);
            bend(myMotor4, dir, pwr);
            PID(pos_pinky, 200);
            bend(myMotor4, dir, pwr);
        }
    }
}

// Function to move desired motor based on PID control
// Params :
//      motor - Desired motor number
//      _dir - direction to move
//      power - power to give motor between 0 and 255
void bend(Adafruit_DCMotor* motor, int _dir, float power)
{
    motor->setSpeed(power);

    if (_dir == 1)                      // Move CW
    {
        motor->run(FORWARD);
        delay(freq);
        motor->run(RELEASE);
        delay(freq);
    }
    else if (_dir == -1)                // Move CCW
    {
        motor->run(BACKWARD);
        delay(freq);
        motor->run(RELEASE);
        delay(freq);
    }
    else                                // Don't move
    {
        myMotor1->run(RELEASE);
        myMotor2->run(RELEASE);
        myMotor3->run(RELEASE);
        myMotor4->run(RELEASE);
    }
}

// Function to stop the motors from moving
void stop()
{
    myMotor1->run(RELEASE);
    myMotor2->run(RELEASE);
    myMotor3->run(RELEASE);
    myMotor4->run(RELEASE);
}

// Function to implement PID position feedback control
// Params :
//      pos - current position
//      goal - desired position
void PID(int pos, int goal)
{
    // Define proportional, integral, and derivative gains
    float kp = 0.1;
    float ki = 0;
    float kd = 0.1;

    // Set time difference 
    long currT = micros();

    float deltaT = ((float)(currT-prevT))/1.0e6;
    prevT = currT;

    // Error between current position and goal
    int e = pos-goal; 

    // Compute derivative 
    float dedt = (e-eprev)/(deltaT);

    // Compute integral 
    eintegral = eintegral + e*deltaT;

    // Compute control signal
    float u = kp*e + kd*dedt + ki*eintegral;

    // Use absolute value for the motor power
    pwr = fabs(u);

    // Motor direction clockwise unless control signal is negative
    dir = 1;

    if (u<0)
    {
        dir = -1;
    }

    // Store previous error
    eprev = e;
}

// ISR for index finger encoder
void readIndexEncoder()
{
    int b_index = digitalRead(ENCB_index);
    if(b_index>0)
    {
        pos_index++;  // CW (finger up)
    }
    else
    {
        pos_index--;  // CCW (finger down)
    }
}

// ISR for Timer1
// Reads encoders for middle, fourth, and pinky fingers
ISR(TIMER1_COMPA_vect)    
{  
    // Read middle finger channel A 
    a_middle = digitalRead(ENCA_middle);

    // RISING edge: from LOW to HIGH
    if (a_middle_prev == LOW && a_middle == HIGH)
    {
        // Read middle finger channel B 
        b_middle = digitalRead(ENCB_middle);

        // Encoder lookup table for CW and CCW
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
    // Save current readings as previous
    a_middle_prev = a_middle;
    b_middle_prev = b_middle;

    // Read fourth finger channel A
    a_fourth = digitalRead(ENCA_fourth);

    // RISING edge: from LOW to HIGH
    if (a_fourth_prev == LOW && a_fourth == HIGH)
    {
        // Read fourth finger channel B 
        b_fourth = digitalRead(ENCB_fourth);

        // Encoder lookup table for CW and CCW
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
    // Save current readings as previous
    a_fourth_prev = a_fourth;
    b_fourth_prev = b_fourth;

    // Read pinky finger channel A
    a_pinky = digitalRead(ENCA_pinky);

    // RISING edge: from LOW to HIGH
    if (a_pinky_prev == LOW && a_pinky == HIGH)
    {
        // Read fourth finger channel B 
        b_pinky = digitalRead(ENCB_pinky);

        // Encoder lookup table for CW and CCW
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
    // Save current readings as previous
    a_pinky_prev = a_pinky;
    b_pinky_prev = b_pinky;
}
