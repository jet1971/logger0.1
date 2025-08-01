#ifndef DETECTVENUE_H
#define DETECTVENUE_H

#include <Arduino.h>

extern double startLineLat;
extern double startLineLng;
extern double startLineLat2;
extern double startLineLng2;

String detectVenue(float currentLatitude, float currentLongitude);

#endif // LOGRECORD_H
