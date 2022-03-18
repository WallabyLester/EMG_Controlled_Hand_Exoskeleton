/********************************************************************
 * Read analog input of Myoware EMG sensor and convert it to voltage 
 * printed to the serial port. 
 ********************************************************************/

int sensor1 = A0;
int sensor2 = A1;
int sensor3 = A2;
int sensor4 = A3;

int sensorVal1 = 0;
int sensorVal2 = 0;
int sensorVal3 = 0;
int sensorVal4 = 0;

String sensorLabel1 = "EMG Sensor 1";
String sensorLabel2 = "EMG Sensor 2";
String sensorLabel3 = "EMG Sensor 3";
String sensorLabel4 = "EMG Sensor 4";

int freq = 1000;    //data collection frequency
bool flag = true;

void setup() 
{
    // initialize serial communication at 9600 bits per second:
    Serial.begin(9600);
}

void loop() 
{
    while(flag)
    {
        // Print EMG sensor labels
        Serial.print(sensorLabel1);     
        Serial.print(",");
        Serial.print(sensorLabel2);
        Serial.print(",");
        Serial.print(sensorLabel3);
        Serial.print(",");
        Serial.println(sensorLabel4);
        flag = false;
    }

    sensorVal1 = analogRead(sensor1);               // Read the input on analog pin
    float voltage1 = sensorVal1 * (5.0 / 1023.0);   // Convert the analog reading to a voltage
    
    sensorVal2 = analogRead(sensor2);
    float voltage2 = sensorVal2 * (5.0 / 1023.0);

    sensorVal3 = analogRead(sensor3);
    float voltage3 = sensorVal3 * (5.0 / 1023.0);

    sensorVal4 = analogRead(sensor4);
    float voltage4 = sensorVal4 * (5.0 / 1023.0);
    
    // Print EMG readings to serial buffer
    Serial.print(voltage1);
    Serial.print(",");
    Serial.print(voltage2);
    Serial.print(",");
    Serial.print(voltage3);
    Serial.print(",");
    Serial.println(voltage4);

    delay(freq);    
  
}
