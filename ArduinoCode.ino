int ethanol_sensor = 1;
int temp_sensor = 2;
int heat_element = 3;

float ethanolVoltage = 0.0;
float tempVoltage = 0.0;

float ethanolReading = 0.0;
float tempReading = 0.0;

float tempValue = 0.0;
float ethanolValue = 0.0;

void setup() {
  Serial.begin(9600); // put your setup code here, to run once:
  pinMode(heat_element, OUTPUT); //sets the digital pin of the heating element as an output
}

void loop() {
  ethanolReading = analogRead(ethanol_sensor);
  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  //ethanolVoltage = map(ethanolReading, 0, 1023, 0, 5000); //Change values depending on the sensor
  //Process value from the sensor into actual ethanol value
  //ethanolValue =
  //Save value - send through Bluetooth
  Serial.println("Ethanol Value: " + ethanolValue);

  tempReading = analogRead(temp_sensor); //Process value from the sensor into actual temp
  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  //tempVoltage = map(tempReading, 0, 1023, 0, 5000); //Change values depending on the sensor
  //Process value from the sensor into actual temperature
  //tempValue =
  Serial.println("Temp Value: " + tempValue); // Print temp to screen for testing but not for final product

  if (tempValue >= 40){
    digitalWrite(heat_element, LOW); //turn heater OFF;
  }
  else{
    digitalWrite(heat_element, HIGH); //turn heater ON;
  }

  delay(1000); //It's in milliseconds
}
