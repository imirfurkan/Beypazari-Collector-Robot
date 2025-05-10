#pragma once

void initLineSensors();
void readLineSensors(float out[5]);
bool allSensorsBlack(const float values[5], float threshold);
