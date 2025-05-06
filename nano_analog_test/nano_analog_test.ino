#include <Arduino.h>

#define SENSOR A0


void setup(){
  Serial.begin(115200);
  pinMode(SENSOR, INPUT);
}


void loop(){
  uint16_t data;
  data = (uint16_t)analogRead(SENSOR);
  Serial.print("Sensor A0:");
  Serial.println(data);
  delay(100);
}
