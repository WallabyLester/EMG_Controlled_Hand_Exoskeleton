/********************************************************************
 * Read analog input of Myoware EMG sensor and convert it to voltage 
 * printed to the serial port. 
 ********************************************************************/

int sensor1Pin = A0;
int freq = 1000;    //data collection frequency
int sensorVal = 0;
String sensorLabel1 = "EMG Sensor 1";
bool flag = true;

void setup() 
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

}

void loop() 
{
  while(flag)
  {
      Serial.println(sensorLabel1);
      flag = false;
  }

  sensorVal = analogRead(sensor1Pin);           // read the input on analog pin 0
  float voltage = sensorVal * (5.0 / 1023.0);   // Convert the analog reading to a voltage
  
  Serial.println(voltage);

  delay(freq);
  
}
