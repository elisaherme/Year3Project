

// First we include the libraries
#include <OneWire.h> 
#include <DallasTemperature.h>

// Data wire is plugged into pin 2 on the Arduino 
#define ONE_WIRE_BUS 2 

// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(ONE_WIRE_BUS); 
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

int ethanol_sensor = 1;
int heat_element = 3;
int threshold = 40; //sweat prodcing threshold temperature
float loadResistance = 1.5; //Put in value of pull down resistance
float controlResistance = 1.5; //Put in value of standar resistance of the sensor at 300ppm, refer to the datasheet
// http://www.figarosensor.com/products/2620pdf.pdf

float ethanolVoltage = 0.0;
float sensorResistance = 0.0;

float ethanolReading = 0.0;
float tempReading = 0.0;

float tempValue = 0.0;
float ethanolValue = 0.0; //measured in ppm

void setup() {
  Serial.begin(9600); //put your setup code here, to run once:
  pinMode(heat_element, OUTPUT); //sets the digital pin of the heating element as an output
  sensors.begin();
}

void loop() {
  //analogRead returns value 0-1023 where the input voltage is 0-5V
  ethanolReading = analogRead(ethanol_sensor);
  //analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  ethanolVoltage = map(ethanolReading, 0, 1023, 0, 5000); //Change values depending on the sensor
  //Process value from the sensor into actual ethanol value
  sensorResistance = ((5000-ethanolVoltage)/ethanolVoltage)*loadResistance;
  ethanolValue = (sensorResistance/controlResistance)*300;
  // need to calibrate the ethanol value to get the control resistance
  //Save value - send through Bluetooth
  Serial.println("Ethanol Value: " + ethanolValue);
  

  sensors.requestTemperatures(); // Send the command to get temperature readings
  //Put temperate sensor into digital pin 2
  tempValue = sensors.getTempCByIndex(0);// Why "byIndex"?  
   // You can have more than one DS18B20 on the same bus.  
   // 0 refers to the first IC on the wire 
  
  Serial.println("Temp Value: " + tempValue); // Print temp to screen for testing but not for final product

  if (tempValue >= threshold){
    digitalWrite(heat_element, LOW); //turn heater OFF;
  }
  else{
    digitalWrite(heat_element, HIGH); //turn heater ON;
  }

  delay(1000); //It's in milliseconds
}

// http://www.figarosensor.com/products/2620pdf.pdf
