#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>

void loadLoggerSettings(); // Function to get logger settings
void saveLoggerSettings();
void saveVenueListVersion();

extern String loggerId; // Default Logger ID, must be exactly 3 characters
extern bool daylightSaving; // Daylight saving time setting
extern int rpmMultiplier; // RPM multiplier setting
extern int tpsMin; // used for tps calibation
extern int tpsMax; // used for tps calibation
extern int venueListVersion;

#endif // SETTINGS_H