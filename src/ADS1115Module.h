#ifndef ADS1115MODULE_H
#define ADS1115MODULE_H

#include <Arduino.h>
#include "ADS1X15.h"

extern ADS1115 ADS1;
extern ADS1115 ADS2;

// Initialize both ADS1115 modules
void setupADS();

// Read and return a compensated voltage based on a divider
double readAndCompensate(ADS1115 &ads, int channel, double r1, double r2);

// Read a voltage-scaled sensor and return it as a scaled uint8_t value
uint8_t readSensor(ADS1115 &ads, int channel, double r1, double r2);

// Read and convert throttle position sensor data to 0-100%
uint8_t readTps(ADS1115 &ads, int channel);

// Map a wideband lambda voltage to a uint8_t representation of lambda*100
uint8_t mapLambda(ADS1115 &ads, int channel, double r1, double r2);

#endif // ADS1115MODULE_H
