#pragma once
#include <NewPing.h>

static constexpr unsigned int SEEKER_THRESHOLD = 70; // cm, for bottle-seeker
const unsigned int OBSTACLE_THRESHOLD = 20;          // cm, for obstacle‐avoid in LINE_SEARCH

/// HC‑SR04 trigger/echo pin mappings //TODO check the current pin mappings
static constexpr int TRIG_LEFT = 31;
static constexpr int ECHO_LEFT = 32;
static constexpr int TRIG_MIDDLE = 33;
static constexpr int ECHO_MIDDLE = 34;
static constexpr int TRIG_RIGHT = 35;
static constexpr int ECHO_RIGHT = 36;

// ─────────────────────────────────────────────────────────
//  Helper: read sonar and treat “0” as “far”
//  inline keyword is used to reduce function call overhead and optimize for the compiler
// ─────────────────────────────────────────────────────────
static inline unsigned int readDistance(NewPing& sonar)
{
  unsigned int cm = sonar.ping_cm();
  return cm ? cm : (SEEKER_THRESHOLD + 1);
}