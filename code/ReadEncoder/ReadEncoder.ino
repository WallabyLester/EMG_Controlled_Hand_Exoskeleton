/*********************************************************************
 * Various brainstorm encoder reading/control sketches
**********************************************************************/

#define inputCLK 4  // digital input (CHA)
#define inputDT 5   // digital input (CHB)

int counter = 0;
int currentStateCLK;
int previousStateCLK;

void setup()
{
    // set encoder pins as inputs
    pinMode (inputCLK, INPUT);
    pinMode (inputDT, INPUT);

    // setup serial monitor
    Serial.begin (9600);

    // read initial state of inputCLK
    previousStateCLK = digitalRead(inputCLK);
}

void loop()
{
    currentStateCLK = digitalRead(inputCLK);

    if (currentStateCLK != previousStateCLK)
    {
        // if inputDT state different than inputCLK state then encoder is counterclockwise
        if (digitalRead(inputDT) != currentStateCLK)
        {
            counter --;
            // if (counter<0)
            // {
            //     counter=0;
            // }
        }
        else
        {
            // clockwise
            counter ++;
            // if (counter>180)
            // {
            //     counter=180;
            // }
        }
        // move motor based on counter value
        previousStateCLK = currentStateCLK;
    }
}

// example for DC motor
# define ENC_COUNT_REV 374

#define ENC_IN 3    // encoder interrupt on interrupt pin 3

volatile long encoderValue = 0;

// one-second interval for measurements
int interval = 1000;  // milliseconds

// counter for milliseconds during interval 
long previousMillis = 0;
long currentMillis = 0;

void setup()
{
    Serial.begin(9600);

    // set encoder as input with internal pullup
    pinMode(ENC_IN, INPUT_PULLUP0);

    // attach interrupt
    attachInterrupt(digitalPinToInterrupt(ENC_IN), updateEncoder, RISING);

    previousMillis = millis();
}

void updateEncoder()
{
    encoderValue++;
}

/*********************************************************************
 * This is for controlling DC motor with encoder
**********************************************************************/

// THIS IS FOR VIEWING ENCODER OUTPUT PWMS
#define ENCA 2  // yellow
#define ENCB 3  // white

int pos = 0;

void setup()
{
    Serial.begin(9600);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
}

void loop()
{
    int a = digitalRead(ENCA);
    int b = digitalRead(ENCA);
    Serial.print(a*5);  // 5 is for readability
    Serial.print(" ");
    Serial.print(b*5);
    Serial.println();
}

// THIS IS FOR READING ENCODER POSITION
#define ENCA 2  // yellow
#define ENCB 3  // white
// the three motor driver pins, different for adafruit
#define PWM 5
#define IN2 6
#define IN1 7

int pos = 0;

void setup()
{
    Serial.begin(9600);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop()
{
    Serial.println(pos);
}

void readEncoder()
{
    int b = digitalRead(ENCB);
    if(b>0)
    {
        pos++;  //CW
    }
    else
    {
        pos--;  //CCW
    }
}

// This is for setting motor movement
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{
    analogWrite(pwm, pwmVal);   // set speed
    if(dir == 1)    // move one way
    {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }
    else if(dir == -1)  // move other way
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
    else    // do not rotate
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
}

void loop()
{
    setMotor(1, 25, PWM, IN1, IN2);
    delay(200);
    Serial.println(pos);
    setMotor(-1, 25, PWM, IN1, IN2);
    delay(200);
    Serial.println(pos);
    setMotor(0, 25, PWM, IN1, IN2);
    delay(200);
    Serial.println(pos);
}

// Controlling motor position using feedback loop
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void loop()
{
    // set target position
    int target = 1200;

    // PID constants
    float kp = 1;
    float kd = 0.025;
    float ki = 0;

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

    // motor power
    float pwr = fabs(u);
    if(pwr>255)
    {
        pwr = 255;
    }

    // motor direction 
    int dir = 1;
    if(u<0)
    {
        dir = -1;
    }

    // signal the motor
    setMotor(dir, pwr, PWM, IN1, IN2);

    // store previous error
    eprev = e;

    Serial.print(target);
    Serial.print(" ");
    Serial.print(pos);
    Serial.println();
}