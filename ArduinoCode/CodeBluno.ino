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

int ethanol_sensor = A0;
int heat_element = 3;
int threshold = 40; //sweat prodcing threshold temperature
// http://www.figarosensor.com/products/2620pdf.pdf

float ethanolVoltage = 0.0;
float sensorResistance = 0.0;

float ethanolReading = 0.0;
float tempReading = 0.0;

float tempValue = 0.0;
double ethanolValue = 0.0; //measured in ppm

void setup() {
  Serial.begin(115200); //put your setup code here, to run once:
  pinMode(heat_element, OUTPUT); //sets the digital pin of the heating element as an output
  sensors.begin();
}

void loop() {
  Serial.print("#");
  //analogRead returns value 0-1023 where the input voltage is 0-5V
  ethanolReading = analogRead(ethanol_sensor);
  //analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  ethanolVoltage = map(ethanolReading, 0, 1023, 0, 5000); //Change values depending on the sensor

  //In percentage
  ethanolValue = pow(10,(ethanolVoltage - 3993)/245.9); //Sensor 1
  Serial.print("|");
  Serial.printIn();

  sensors.requestTemperatures(); // Send the command to get temperature readings
  //Put temperate sensor into digital pin 2
  tempValue = sensors.getTempCByIndex(0);// Why "byIndex"?
   // You can have more than one DS18B20 on the same bus.
   // 0 refers to the first IC on the wire

  if (tempValue >= threshold){
    digitalWrite(heat_element, LOW); //turn heater OFF;
  }
  else{
    digitalWrite(heat_element, HIGH); //turn heater ON
  }

  delay(10000); //It's in milliseconds
}
