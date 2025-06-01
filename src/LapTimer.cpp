#include "LapTimer.h"

LapTimer::LapTimer(double lat1, double lon1, double lat2, double lon2, double triggerDistance, uint32_t debounceMs)
    : lineLat1(lat1), lineLon1(lon1), lineLat2(lat2), lineLon2(lon2),
      triggerDistance(triggerDistance), debounceTime(debounceMs) {}

void LapTimer::latLonToXY(double lat, double lon, double &x, double &y)
{
    if (!originSet)
    {
        originLat = lat;
        originLon = lon;
        originSet = true;
        Serial.println("Origin set!");
    }

    const double R = 6371000.0; // Earth's radius in meters
    double dLat = radians(lat - originLat);
    double dLon = radians(lon - originLon);

    double meanLat = radians((lat + originLat) / 2.0);
    x = R * dLon * cos(meanLat);
    y = R * dLat;
}

double LapTimer::perpendicularDistance(double x1, double y1, double x2, double y2, double px, double py)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    double lengthSquared = dx * dx + dy * dy;

    if (lengthSquared == 0)
    {
        // Line segment is a point
        return sqrt((px - x1) * (px - x1) + (py - y1) * (py - y1));
    }

    double t = ((px - x1) * dx + (py - y1) * dy) / lengthSquared;
    t = max(0.0, min(1.0, t)); // Clamp t to the range [0, 1]

    double closestX = x1 + t * dx;
    double closestY = y1 + t * dy;

    return sqrt((px - closestX) * (px - closestX) + (py - closestY) * (py - closestY));
}

bool LapTimer::segmentsIntersect(double x1, double y1, double x2, double y2,
                                 double x3, double y3, double x4, double y4)
{
    auto ccw = [](double ax, double ay, double bx, double by, double cx, double cy)
    {
        return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax);
    };

    bool ccw1 = ccw(x1, y1, x3, y3, x4, y4);
    bool ccw2 = ccw(x2, y2, x3, y3, x4, y4);
    bool ccw3 = ccw(x1, y1, x2, y2, x3, y3);
    bool ccw4 = ccw(x1, y1, x2, y2, x4, y4);

    // Debug: Print ccw results
    // Serial.println("----- segmentsIntersect Debug -----");
    // Serial.print("ccw1: ");
    // Serial.println(ccw1);
    // Serial.print("ccw2: ");
    // Serial.println(ccw2);
    // Serial.print("ccw3: ");
    // Serial.println(ccw3);
    // Serial.print("ccw4: ");
    // Serial.println(ccw4);

    return ccw1 != ccw2 && ccw3 != ccw4;
}

bool LapTimer::checkLap(double lat, double lon, uint32_t timestamp)
{
    static double prevX = 0.0, prevY = 0.0;
    double currX, currY;

    latLonToXY(lat, lon, currX, currY);

    double x3, y3, x4, y4;
    latLonToXY(lineLat1, lineLon1, x3, y3);
    latLonToXY(lineLat2, lineLon2, x4, y4);

    // Debug: Print current and previous positions
    // Serial.println("----- Debug Info -----");
    // Serial.print("Current Position: X=");
    // Serial.print(currX);
    // Serial.print(", Y=");
    // Serial.println(currY);
    // Serial.print("Previous Position: X=");
    // Serial.print(prevX);
    // Serial.print(", Y=");
    // Serial.println(prevY);

    // Debug: Print line coordinates
    // Serial.print("Line Start: X=");
    // Serial.print(x3);
    // Serial.print(", Y=");
    // Serial.println(y3);
    // Serial.print("Line End: X=");
    // Serial.print(x4);
    // Serial.print(", Y=");
    // Serial.println(y4);

    // Calculate perpendicular distance to the line
    double distToLine = perpendicularDistance(x3, y3, x4, y4, currX, currY);

    // Debug: Print distance to line and trigger distance
    // Serial.print("Distance to Line: ");
    // Serial.println(distToLine);
    // Serial.print("Trigger Distance: ");
    // Serial.println(triggerDistance);

    if (distToLine > triggerDistance)
    {
        wasOverLine = false; // Reset when far from the line
        prevX = currX;
        prevY = currY;

        // Debug: Print reset state
        // Serial.println("Too far from line. Resetting wasOverLine.");
        // Serial.println();
        return false;
    }

    bool crossed = segmentsIntersect(prevX, prevY, currX, currY, x3, y3, x4, y4);

    // Debug: Print crossing state
    Serial.print("Crossed Line: ");
    Serial.println(crossed ? "Yes" : "No");

    prevX = currX;
    prevY = currY;

    // Debug: Print updated previous position
    // Serial.print("Updated Previous Position: X=");
    // Serial.print(prevX);
    // Serial.print(", Y=");
    // Serial.println(prevY);

    if (crossed && !wasOverLine)
    {
        // Ignore the first crossing
        if (ignoreFirstLap)
        {
            Serial.println("Ignoring first crossing.");
            ignoreFirstLap = false; // Clear the flag after the first crossing
            wasOverLine = true;     // Set to true to avoid immediate re-triggering
            lastLapStart = timestamp; // Reset the last lap start time
            return false;
        }
        // Debug: Print debounce check
        Serial.print("Debounce Check: Time since last lap = ");
        Serial.println(timestamp - lastLapStart);

        if (timestamp - lastLapStart >= debounceTime)
        {
            lastLapTime = timestamp - lastLapStart;
            lastLapStart = timestamp;
            wasOverLine = true;

            lapNumber++;
            if (lapNumber < MAX_LAPS)
            {
                lapTimes[lapNumber - 1] = lastLapTime;
            }

            // Debug: Print lap details
            Serial.println("Lap crossed!");
            Serial.print("Lap number: ");
            Serial.println(lapNumber);
            Serial.print("Last lap time: ");
            Serial.println(lastLapTime);
            Serial.print("Total laps: ");
            Serial.println(lapNumber);
            Serial.print("Lap times: ");
            for (uint8_t i = 0; i < lapNumber; i++)
            {
                Serial.print(lapTimes[i]);
                Serial.print(" ");
            }
            Serial.println();

            wasOverLine = false; // Reset after processing the lap
            return true;
        }
        else
        {
            // Debug: Print debounce failure
            Serial.println("Debounce time not met. Lap not triggered.");
        }
    }

    wasOverLine = crossed; // Update state based on crossing

    // Debug: Print updated wasOverLine state
    Serial.print("Updated wasOverLine: ");
    Serial.println(wasOverLine ? "True" : "False");

    return false;
}


