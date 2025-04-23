#include <Arduino.h>
#include "ultrasonic.h"

// // defines pins numbers
// const int trigPin = 9;
// const int echoPin = 10;

// safe max duration in µs so that there is no overflow
const unsigned long MAX_DURATION = 50000;

void ultrasonicSetup(int trigPin, int echoPin)
{
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
}

float measureDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration =
      pulseIn(echoPin, HIGH); // unsigned is better for embedded systems with little memory. avoid
  // mixing signed and unsigned numbers though.

  if (duration > MAX_DURATION) //   if (duration == 0 || duration > MAX_DURATION)
  {
    Serial.println("Object too far or no echo!");
    // return NAN; // or some error value
  }

  Serial.print("Raw duration (µs): ");
  Serial.println(duration);

  return duration * 0.034 / 2;
}