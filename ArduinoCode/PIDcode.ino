// First we include the libraries
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

double threshold, tempValue, Output;
//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&tempValue, &Output, &threshold, consKp, consKi, consKd, DIRECT);
PID_ATune aTune(&tempValue, &Output);

int val = 0;
int is_tuning = 1;

int heat_element = 3;
int ethanol_sensor = A0;
float ethanolVoltage = 0.0;
float ethanolReading = 4.0;
double ethanolValue = 3.0; //measured in ppm

void setup() {
  Serial.begin(115200); //put your setup code here, to run once:
  pinMode(heat_element, OUTPUT); //sets the digital pin of the heating element as an output
  sensors.begin();

  //initialize the variables we're linked to
  threshold = 40;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop() {

  //analogRead returns value 0-1023 where the input voltage is 0-5V
  ethanolReading = analogRead(ethanol_sensor);
  //analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  ethanolVoltage = map(ethanolReading, 0, 1023, 0, 5000); //Change values depending on the sensor

  Serial.print("ethanol value = ");
  //In percentage
  ethanolValue = pow(10,(ethanolVoltage - 3578.9)/328.0); //Sensor 2
  Serial.println(ethanolValue);

  Serial.print("temp = ");
  sensors.requestTemperatures(); // Send the command to get temperature readings
  //Put temperate sensor into digital pin 2
  tempValue = sensors.getTempCByIndex(0);// Why "byIndex"?
   // You can have more than one DS18B20 on the same bus.
   // 0 refers to the first IC on the wire
  Serial.println(tempValue);

  if (is_tuning){                                                             // Check whether tunner is turned on                                                      // TUNE
    val = 0;
    val = (aTune.Runtime());                                                // Run the Auto tunner and check whether it is finished the tuning
    if(val != 0){                                                             // If tuning is done
      is_tuning = 0;                                                          // Turn off the tunner
      consKp = aTune.GetKp();
      consKi = aTune.GetKi();
      consKd = aTune.GetKd();
      myPID.SetTunings(consKp, consKi, consKd);                       // Set the tunned constants to the PID
      PID.SetMode(AUTOMATIC);
    }
  }
  else{
    PID.Compute();
    analogWrite(heat_element,Output);
  }
  //double gap = abs(threshold-tempValue); //distance away from setpoint
  //if(gap<5)
  //{  //we're close to setpoint, use conservative tuning parameters
  //myPID.SetTunings(consKp, consKi, consKd);
  //}
  //else
  //{
     //we're far from setpoint, use aggressive tuning parameters
     //myPID.SetTunings(aggKp, aggKi, aggKd);
  //}

  //myPID.Compute();
  //analogWrite(heat_element,Output);
}
