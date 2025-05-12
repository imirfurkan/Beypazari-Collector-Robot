#pragma once

#include <stdint.h>

// Call once in setup()
void setupTOF();

// Enable / disable the VL53L0X hardware interrupt
void enableTOFInterrupt();
void disableTOFInterrupt();

// Adjust the trigger window (low_mm = trigger-below; high_mm = stop-trigger-above)
void setTOFThresholds(uint16_t low_mm, uint16_t high_mm);
