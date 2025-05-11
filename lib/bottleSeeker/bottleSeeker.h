// BottleSeeker.h
#pragma once

#include <Arduino.h>
#include <NewPing.h>

// ─────────────────────────────────────────────────────────
//  Helper: read sonar and treat “0” as “far”
//  inline keyword is used to reduce function call overhead and optimize for the compiler
// ─────────────────────────────────────────────────────────
static constexpr unsigned int DIST_THRESHOLD = 70;
static inline unsigned int readDistance(NewPing& sonar)
{
  unsigned int cm = sonar.ping_cm();
  return cm ? cm : (DIST_THRESHOLD + 1);
}

/**
 * @brief Initialize bottle-seeker motors and sensors.
 */
void BottleSeeker_setup();

/**
 * @brief Run one step of the bottle-seeker FSM.
 * @return true when a bottle is positioned for gripping; false otherwise.
 */
bool BottleSeeker_loop();
