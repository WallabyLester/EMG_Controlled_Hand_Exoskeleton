/********************************************************************
 * Read all encoders.
*********************************************************************/

// Define encoder channel pins
#define ENCA_index 2  
#define ENCB_index 3
#define ENCA_middle 4  
#define ENCB_middle 5
#define ENCA_fourth 7  
#define ENCB_fourth 6
#define ENCA_pinky 8  
#define ENCB_pinky 9

// Initialize encoder counters
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

void setup()
{
    // Initialize serial communication at 9600 bits per second
    Serial.begin(9600);
    
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
    // Read encoder channels for current state
    a_middle_prev = digitalRead(ENCA_middle);
    b_middle_prev = digitalRead(ENCB_middle);

    a_fourth_prev = digitalRead(ENCA_fourth);
    b_fourth_prev = digitalRead(ENCA_fourth);

    a_pinky_prev = digitalRead(ENCA_pinky);
    b_pinky_prev = digitalRead(ENCA_pinky);

    // Print encoder counts 
    Serial.print(pos_index); 
    Serial.print(" ");
    Serial.print(pos_middle);
    Serial.print(" ");
    Serial.print(pos_fourth);
    Serial.print(" ");
    Serial.print(pos_pinky);
    Serial.println();
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
        if (a_fourth_prev == LOW && b_middle_prev == LOW && a_fourth == LOW && b_fourth == HIGH)
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
