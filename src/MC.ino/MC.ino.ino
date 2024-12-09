
#include <Wire.h>
#include "MS5837.h"
#include<Servo.h>
MS5837 sensor;
uint8_t thrusterStatus = 0;
#define depthSensorConnected 1
double surface_pressure = 0;

void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);
#if depthSensorConnected
  Wire.begin();
  /*while (!sensor.init())
  {
    delay(1000);
  }*/
    //Serial.println("Hello");
    sensor.init();
    //Serial.println("Hello");
    sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(1000); // kg/m^3 (freshwater, 1029 for seawater)
  //sensor.setModel(MS5837::MS5837_30BA);
  //sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
#endif

}

void loop() {
  // put your main code here, to run repeatedly:
  #if depthSensorConnected
//  sensor.read(); //get sensor data
  double pressure = sensor.depth() - surface_pressure;
  double pressur2= sensor.pressure() - surface_pressure;
//  if((pressure) < 930 || (pressure > 1200))
//    pressure = pre_depth;
//  else 
//    pre_depth = pressure;
    
  String dat;
  dat += 'D';
  dat += float(pressure);
  dat += '#';
  dat += 'P';
  dat += float(pressur2);
  dat += '#';
  delay(50);
  //Serial.println(pressure);
   Serial.println(dat); //send sensor data

#endif
 
}
