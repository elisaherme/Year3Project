#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

int heat_element = 3;
int ethanol_sensor = A0;
float ethanolVoltage = 0.0;
float ethanolReading = 4.0;
double ethanolValue = 3.0; //measured in ppm

byte ATuneModeRemember=2;
double input, output, setpoint=34;
double kp=2,ki=0.5,kd=2;

double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;

boolean tuning = false;
unsigned long  modelTime, serialTime;

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&input, &output);

//set to false to connect to the real world
boolean useSimulation = true;

void setup()
{
  if(useSimulation)
  {
    for(byte i=0;i<50;i++)
    {
      theta[i]=outputStart;
    }
    modelTime = 0;
  }
  //Setup the pid
  myPID.SetMode(AUTOMATIC);

  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }

  serialTime = 0;
  Serial.begin(115200);
  pinMode(heat_element, OUTPUT); //sets the digital pin of the heating element as an output
  sensors.begin();

}

void loop()
{
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
  input = sensors.getTempCByIndex(0);// Why "byIndex"?
   // You can have more than one DS18B20 on the same bus.
   // 0 refers to the first IC on the wire
  Serial.println(input);

  unsigned long now = millis();

  if(!useSimulation)
  { //pull the input in from the real world
    input = analogRead(0);
  }

  if(tuning)
  {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  }
  else myPID.Compute();

  if(useSimulation)
  {
    theta[30]=output;
    if(now>=modelTime)
    {
      modelTime +=100;
      DoModel();
    }
  }
  else
  {
     analogWrite(0,output);
  }

  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
}

void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}


void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  }
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read();
   Serial.flush();
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}

void DoModel()
{
  //cycle the dead time
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  //compute the input
  input = (kpmodel / taup) *(theta[0]-outputStart) + input*(1-1/taup) + ((float)random(-10,10))/100;

}
