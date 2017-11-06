// fsr 
int fsrPin1 = 1; // the FSR and 10K pulldown are connected to a0 
int fsrPin2 = 2; 
int fsrPin3 = 3; 
int fsrReading =0 ; // the analog reading from the FSR resistor divider 
int fsrVoltage =0; // the analog reading converted to voltage 
unsigned long fsrResistance=0; // The voltage converted to resistance, can be very big so make "long" 
unsigned long fsrConductance=0; 
long fsrForce =0; // Finally, the resistance converted to force 
 
// speed 
volatile byte revolutions=0; 
unsigned int rpm=0; 
float v; 
unsigned long timeold=0; 
unsigned long interval=0; 
 
void setup(void) 
{ 
    Serial.begin(9600); // We'll send debugging information via the Serial monitor 
    attachInterrupt(0, magnet_detect, RISING);//Initialize the intterrupt pin (Arduino digital pin 2) 
} 
  
void loop(void) 
{ 
                      Serial.print("#");
                      for(int i=fsrPin1; i<=fsrPin3; i++) 
                      { 
                          fsrReading = analogRead(i); 
                          //Serial.print("FSR "); 
                          //Serial.print(i); 
                          //Serial.print('\n'); 
                          //Serial.print("Analog reading = "); 
                          //Serial.println(fsrReading); 
                          // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV) 
                          fsrVoltage = map(fsrReading, 0, 1023, 0, 5000); 
                          //Serial.print("Voltage reading in mV = "); 
                          //Serial.println(fsrVoltage); 
                          if (fsrVoltage == 0)  
                              { 
                                 // Serial.println("No pressure"); 
                              }  
                          else  
                              { 
                              // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V 
                              // so FSR = ((Vcc - V) * R) / V 
                               
                              fsrResistance = 5000 - fsrVoltage; // fsrVoltage is in millivolts so 5V = 5000mV 
                              fsrResistance *= 10000; // 10K resistor 
                              fsrResistance /= fsrVoltage; 
                              //Serial.print("FSR resistance in ohms = "); 
                              //Serial.println(fsrResistance); 
                              fsrConductance = 1000000; // we measure in micromhos so 
                              fsrConductance /= fsrResistance; 
                              //Serial.print("Conductance in microMhos: "); 
                              //Serial.println(fsrConductance); 
                              // Use the two FSR guide graphs to approximate the force 
                              if (fsrConductance <= 1000) 
                                  { 
                                      fsrForce = fsrConductance / 80; 
                                      //Serial.print("Force in Newtons: "); 
                                      //Serial.print("|"); 
                                      Serial.print(fsrForce); 
                                      Serial.println(";"); 
                                  }  
                              else 
                                  { 
                                      fsrForce = fsrConductance - 1000; 
                                      fsrForce /= 30; 
                                      //Serial.print("Force in Newtons: "); 
                                     // Serial.print("|"); 
                                      Serial.print(fsrForce); 
                                      Serial.println(";"); 
                                  } 
                            } 
                        //Serial.println("--------------------"); 
                        delay(1000); 
                    } 
                    //speed 
                    //Serial.println("velocity: "); 
                    if (revolutions >= 1) 
                      { 
                      interval = millis() - timeold; 
                      rpm = 60000/interval; 
                      v = (0.6) * rpm * 0.1047; 
                      timeold = millis(); 
                      revolutions = 0; 
                      //Serial.println(rpm); 
                      //Serial.println(interval); 
                      //Serial.print("|"); 
                      Serial.print(v); 
                      Serial.println(";"); 
                      } 
                    else 
                      { 
                      v = 0; 
                     // Serial.print("|"); 
                      Serial.print(v); 
                      Serial.println(";"); 
                      } 
                      //Serial.println("--------------------"); 
 Serial.println("~");
                       
                       
} 
                     void magnet_detect()//This function is called whenever a magnet/interrupt is detected by the arduino 
                     { 
                         revolutions++; 
                         Serial.print('\n'); 
                    }
