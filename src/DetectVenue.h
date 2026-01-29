#ifndef DETECTVENUE_H
#define DETECTVENUE_H

#include <Arduino.h>

extern double startLineLat;
extern double startLineLng;
extern double startLineLat2;
extern double startLineLng2;

struct Venue
{
    String name;
    double lat1;
    double lng1;
    double lat2;
    double lng2;
    double radius; // detection tolerance in meters
    String code;     // character to return, 1 for Work, 2 for Home, etc.
};
extern int loadedCount;
extern Venue loadedVenues[]; // Adjust size if needed

String detectVenue(float currentLatitude, float currentLongitude);
void saveVenuesToJson();
void loadVenuesFromJson();

#endif // LOGRECORD_H
