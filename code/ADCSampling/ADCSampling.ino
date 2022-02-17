int numSamples = 0;
long t, t0;

void setup()
{
    Serial.begin(115200);
    // pinMode(A0, INPUT);
    t0 = micros();

    ADCSRA = 0;                 // clear ADCSRA and ADCSRB registers
    ADCSRB = 0;         
    ADMUX |= (0 & 0x07);        // set A0 analog input pins
    ADMUX |= (1 << REFS0);      // set reference voltage
    ADMUX |= (1 << ADLAR);      // left align ADC value to 8 bits from ADCH register

    // sampling rate = <ADC clock> / <prescaler> / <conversion clock cycles>
    // Arduino Uno -> ADC clock: 16 MHz; conversion clock: 13 clock cycles
    // ADCSRA |= (1 << ADPS2) | (1 << ADPS0);  // 32 prescaler to get 38.5 KHz
    ADCSRA |= (1 << ADPS2); // 16 prescaler to get 76.9 KHz
    // ADCSRA |= (1 << ADPS1) | (1 << ADPS0);  // 8 prescaler to get 153.8 KHz

    ADCSRA |= (1 << ADATE); // enable auto trigger
    ADCSRA |= (1 <<ADIE);   // enable interrupts when measurement complete
    ADCSRA |= (1 << ADEN);  // enable ADC
    ADCSRA |= (1 << ADSC);  // start ADC measurements
}

ISR(ADC_vect)
{
    byte x = ADCH;  // read 8 bit value from ADC
    numSamples++;
}

void loop()
{
    if (numSamples>=1000)
    {
        t = micros()-t0;   // elapsed time

        Serial.print("Sampling frequency: ");
        Serial.print((float)1000000/t);
        Serial.println(" KHz");
        delay(2000);

        // restart 
        t0 = micros();
        numSamples=0;
    }
}