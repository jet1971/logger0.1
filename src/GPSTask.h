#ifndef GPSTASK_H
#define GPSTASK_H

#include <Arduino.h>
#include <TinyGPSPlus.h>

void gpsTask(void *parameter);

// Declare variables as extern
extern volatile double currentLat, currentLng, currentSpeed;
extern volatile int day, year, month, hour, minute;
extern volatile bool gpsValid;

// extern TinyGPSPlus gps; // Declare the GPS object as extern
#endif
