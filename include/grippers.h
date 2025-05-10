#pragma once
#include <Arduino.h>

/// Call once in your sketch’s setup() to init pins, servos, and stepper.
void Grippers_setup();

/// Call repeatedly in loop(); returns true once one full grab+store cycle completes.
bool Grippers_loop();
