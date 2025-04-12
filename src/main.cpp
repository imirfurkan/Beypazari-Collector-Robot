#include <Arduino.h>
#include "ultrasonic.h"

void setup()
{
  Serial.begin(9600);
  ultrasonicSetup();
}

void loop()
{
  float distance = measureDistance();
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(500);
}