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
    int b = digitalRead(ENCB);
    Serial.print(a*5);  // 5 is for readability
    Serial.print(" ");
    Serial.print(b*5);
    Serial.println();
}
