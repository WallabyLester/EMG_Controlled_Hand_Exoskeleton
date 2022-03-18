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

void setup()
{
    Serial.begin(9600);
    
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

    Serial.print(pos_index); 
    Serial.print(" ");
    Serial.print(pos_middle);
    Serial.print(" ");
    Serial.print(pos_fourth);
    Serial.print(" ");
    Serial.print(pos_pinky);
    Serial.println();
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
