#include <Arduino.h>

// defines pins numbers
const int trigPin = 9;
const int echoPin = 10;

void ultrasonicSetup()
{
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

float measureDistance()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH);  //unsigned is better for embedded systems with little memory. avoid mixing signed and unsigned numbers though.
  return duration * 0.034 / 2;
}