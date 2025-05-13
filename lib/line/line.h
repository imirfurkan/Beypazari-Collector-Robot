#pragma once

#include <Arduino.h>
#include "ultrasonic.h"
#include "motors.h"

// ────────────────────────────────────────────────
//  IR sensor pins (define in line.h if you prefer)
// ────────────────────────────────────────────────
static constexpr int L0 = A5;
static constexpr int L1 = A4;
static constexpr int L2 = A3;
static constexpr int L3 = A2;
static constexpr int L4 = A1;
static constexpr int L5 = A0;

void updateObstacleAvoid();
/**
 * @brief Initialize line-follower sensors and motor driver.
 * Call once in your sketch setup().
 */
void Line_setup();

/**
 * @brief Run one iteration of the line-search/line-follow state machine.
 * Call repeatedly in your sketch loop().
 */
bool Line_loop();
