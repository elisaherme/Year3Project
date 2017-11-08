int ethanol_sensor = 1;
int temp_sensor = 2;
int heat_element = 3;

float tempValue = 0.0;
float ethanolValue = 0.0;

void setup() {
  Serial.begin(9600); // put your setup code here, to run once:
  pinMode(heat_element, OUTPUT); //sets the digital pin of the heating element as an output
}

void loop() {
  ethanolValue = analogRead(ethanol_sensor); //Process value from the sensor into actual ethanol value
  //Save value - send through Bluetooth
  Serial.println("Ethanol Value: " + ethanolValue);
  tempValue = analogRead(temp_sensor); //Process value from the sensor into actual temp
  Serial.println("Temp Value: " + tempValue); // Print temp to screen for testing but not for final product
  if (tempValue >= 40){
    digitalWrite(heat_element, LOW); //turn heater OFF;
  }
  else{
    digitalWrite(heat_element, HIGH); //turn heater ON;
  }
  delay(1000); //It's in milliseconds
}
