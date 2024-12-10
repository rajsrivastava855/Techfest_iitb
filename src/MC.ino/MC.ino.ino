#include <Wire.h>
#include "MS5837.h"
#include<Servo.h>
MS5837 sensor;
uint8_t thrusterStatus = 0;
#define depthSensorConnected 1
double surface_pressure = 0;

void sendFloat(float value) {
  Serial.print("#A");  // Send identifier for a float
  Serial.write((uint8_t*)&value, sizeof(value));  // Send float as 4 bytes
  Serial.println();
}

void receiveArray() {
  int arraySize = 0;
  if (Serial.available() >= sizeof(int)) {
    // Read array size
    Serial.readBytes((char*)&arraySize, sizeof(int));

    float receivedArray[arraySize];
    for (int i = 0; i < arraySize; i++) {
      while (Serial.available() < sizeof(float));  // Wait for float
      Serial.readBytes((char*)&receivedArray[i], sizeof(float));
    }

    // Print the received array for verification
    Serial.println("Array Received:");
    for (int i = 0; i < arraySize; i++) {
      Serial.print("Element ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(receivedArray[i], 4);
    }
  }
}
void setup() {
  
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
  if (Serial.available() > 0) {
    // Read incoming command
    String command = Serial.readStringUntil('\n');  // Read until newline

    if (command == "#A") {
      // Send a float value when #A command is received
      float testFloat = 3.1415;  // Example float to send
      sendFloat(testFloat);
      Serial.println("Float Sent.");
    }
    else if (command == "#B") {
      // Receive an array of floats when #B command is received
      Serial.println("Waiting to receive array...");
      receiveArray();
    }
  }
 
}
