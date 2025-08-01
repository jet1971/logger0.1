#ifndef AIRTEMPERATUREFUNCTION_H
#define AIRTEMPERATUREFUNCTION_H

#include <Arduino.h>
#include "ADS1X15.h"

extern float airTemperatureVoltagePoints[];

int interpolateAirTemperature(ADS1115 &ads, int channel);
void loadAirTemperatureCalibration();
void saveAirTemperatureCalibration();

#endif