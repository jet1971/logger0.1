#ifndef TEMPERATUREFUNCTION_H
#define TEMPERATUREFUNCTION_H

#include <Arduino.h>
#include "ADS1X15.h"

extern float voltagePoints[]; 

int interpolateEngineTemperature(ADS1115 &ads, int channel, double r1, double r2);
void loadEngineTemperatureCalibration();
void saveEngineTemperatureCalibration();

#endif