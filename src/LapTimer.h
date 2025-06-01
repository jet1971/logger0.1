#ifndef LAP_TIMER_H
#define LAP_TIMER_H

#include <Arduino.h>
#include <math.h>


#define MAX_LAPS 50


class LapTimer
{
public:
    LapTimer(double lat1, double lon1, double lat2, double lon2, double triggerDistance = 5.0, uint32_t debounceMs = 10000);

    // Calculate perpendicular distance to the line
    double perpendicularDistance(double x3, double y3, double x4, double y4, double currX, double currY);
    bool checkLap(double lat, double lon, uint32_t timestamp);
    uint32_t getLastLapTime() const { return lastLapTime; }

    bool updateFramFilenameIfNewBest(char *fileName, size_t length);

private:
    double lineLat1,
        lineLon1;
    double lineLat2, lineLon2;
    double triggerDistance;
    uint32_t debounceTime;

    double originLat, originLon;
    bool originSet = false;
    bool wasOverLine = false;
    bool ignoreFirstLap = true; // Ignore the first crossing

    uint32_t lastLapStart = 0;
    uint32_t lastLapTime = 0;
    uint8_t lapNumber = 0;
    uint32_t fastestLapTime = 999999999; // Initialize to a large value otherwise has no starting value
    uint8_t fastestLapNumber = 0; // Store the lap number of the fastest lap
   

    void latLonToXY(double lat, double lon, double &x, double &y);
    bool segmentsIntersect(double x1, double y1, double x2, double y2,
                           double x3, double y3, double x4, double y4);
};

#endif
