int ethanol_sensor = 1;
int temp_sensor = 2;
int heat_element = 3;

float tempValue = 0.0;
float ethanolValue = 0.0;

void setup() {
  Serial.begin(9600);// put your setup code here, to run once:

}

void loop() {
  ethanolValue = analogRead(ethanol_sensor); //Process value from the sensor into actual ethanol value
  //Save value - send through Bluetooth
  Serial.println("Ethanol Value: " + ethanolValue);
  tempValue = analogRead(temp_sensor); //Process value from the sensor into actual temp
  Serial.println("Temp Value: " + tempValue)// Print to screen for testing but not for final product1
  if (tempValue > 40){
    turn heater OFF;
  }
  else{
    turn heater ON;
  }
  delay(1);
}
