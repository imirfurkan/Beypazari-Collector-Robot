#pragma once
#include <NewPing.h>

static constexpr unsigned int SEEKER_THRESHOLD_SIDES = 50; // cm, for bottle-seeker
static constexpr unsigned int SEEKER_THRESHOLD_MIDDLE = SEEKER_THRESHOLD_SIDES + 15; // cm
static constexpr unsigned int GRIPPER_THRESHOLD = 20;  // cm, for bottle-seeker to stop
static constexpr unsigned int OBSTACLE_THRESHOLD = 10; // cm, for obstacle‐avoid in LINE_SEARCH

/// HC‑SR04 trigger/echo pin mappings //TODO check the current pin mappings
static constexpr int TRIG_LEFT = 48;
static constexpr int ECHO_LEFT = 49;
static constexpr int TRIG_MIDDLE = 44;
static constexpr int ECHO_MIDDLE = 45;
static constexpr int TRIG_RIGHT = 46;
static constexpr int ECHO_RIGHT = 47;

// ─────────────────────────────────────────────────────────
//  Helper: read sonar and treat “0” as “far”
//  inline keyword is used to reduce function call overhead and optimize for the compiler
// ─────────────────────────────────────────────────────────
static inline unsigned int readDistance(NewPing& sonar)
{
  unsigned int cm = sonar.ping_cm();
  return cm ? cm : (999);
}