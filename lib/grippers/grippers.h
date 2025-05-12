#pragma once
#include <Arduino.h>

extern uint8_t bottleCount;

/// Call once in your sketch’s setup() to init pins, servos, and stepper.
void Grippers_setup();

/// Call repeatedly in loop(); returns true once one full grab+store cycle completes.
bool Grippers_loop();

/// Returns true if the last Grippers_loop() cycle that returned true was a reject.
bool bottleRejected();

// New public functions to expose gripper actions
void Grippers_openClaws();
void Grippers_closeClaws(); 
void Grippers_lowerElbows();
void Grippers_raiseElbows();