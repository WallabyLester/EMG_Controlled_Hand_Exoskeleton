#include "TimerOne.h"


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

void setup()
{
    Serial.begin(9600);
    Timer1.initialize();
    
    pinMode(ENCA_index, INPUT);
    pinMode(ENCB_index, INPUT);
    pinMode(ENCA_middle, INPUT);
    pinMode(ENCB_middle, INPUT);
    pinMode(ENCA_fourth, INPUT);
    pinMode(ENCB_fourth, INPUT);
    pinMode(ENCA_pinky, INPUT);
    pinMode(ENCB_pinky, INPUT);


    attachInterrupt(digitalPinToInterrupt(ENCA_index), readIndexEncoder, RISING);
    Timer1.attachInterrupt(readMiddleEncoder);
//    attachInterrupt(digitalPinToInterrupt(ENCA_middle), readMiddleEncoder, RISING);
//    attachInterrupt(digitalPinToInterrupt(ENCA_fourth), readFourthEncoder, RISING);
//    attachInterrupt(digitalPinToInterrupt(ENCA_pinky), readPinkyEncoder, RISING);
}

void loop()
{
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

void readMiddleEncoder()
{
    a_middle = digitalRead(ENCA_middle);

    if (a_middle_prev != a_middle)
    {
      int b_middle = digitalRead(ENCB_middle);
      if(b_middle>0)
      {
          pos_middle++;  //CW (up)
      }
      else
      {
          pos_middle--;  //CCW (down)
      }
    }
    a_middle_prev = a_middle;
}

void readFourthEncoder()
{
    int b_fourth = digitalRead(ENCB_fourth);
    if(b_fourth>0)
    {
        pos_fourth++;  //CW (up)
    }
    else
    {
        pos_fourth--;  //CCW (down)
    }
}

void readPinkyEncoder()
{
    int b_pinky = digitalRead(ENCB_pinky);
    if(b_pinky>0)
    {
        pos_pinky++;  //CW (up)
    }
    else
    {
        pos_pinky--;  //CCW (down)
    }
}
