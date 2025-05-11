// BottleSeeker.h
#pragma once
#include <Arduino.h>

/**
 * @brief Initialize bottle-seeker motors and sensors.
 */
void BottleSeeker_setup();

/**
 * @brief Run one step of the bottle-seeker FSM.
 * @return true when a bottle is positioned for gripping; false otherwise.
 */
bool BottleSeeker_loop();
