#pragma once
#include <Arduino.h>

void initServo(int servoPin);
void initTof();
void updateServoDirection(int stepAngle);
void writeServoAngle();
uint16_t readDistance();
bool objectDetected(uint16_t distance, uint16_t threshold);
void enterPause();
bool isPaused();
bool pauseDone(unsigned long duration);
void resetPause();
int getCurrentAngle();