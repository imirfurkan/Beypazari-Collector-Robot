// tof.h
#pragma once

#include <stdint.h>
#include <Arduino.h>

// Flag set by the ISR when the VL53L0X threshold is crossed
extern volatile bool triggeredObstacle;

uint16_t readTOF();
// Call once in setup()
void setupTOF();

// Enable / disable the VL53L0X hardware interrupt
void enableTOFInterrupt();
void disableTOFInterrupt();

// Clear the VL53L0X’s own interrupt latch so you can get another FALLING edge
void clearTOFInterrupt();

// Adjust the trigger window (low_mm = trigger-below; high_mm = stop-trigger-above)
void setTOFThresholds(uint16_t low_mm, uint16_t high_mm);
