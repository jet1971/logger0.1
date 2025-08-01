#include "LapTimer.h"
#include "FramLogHeader.h"
#include "FRAMModule.h"

bool LapTimer::updateFramFilenameIfNewBest(char *fileName, size_t length)
{
    if (length < 32)
        return false; // Safety check: prevent overflow

    // Convert fastest lap number and time to 2-digit and 6-digit strings
    char lapNumStr[3];
    snprintf(lapNumStr, sizeof(lapNumStr), "%02u", fastestLapNumber); // format fastestLapNumber as 2-digit string, copy to lapNumStr array(buffer)

    char lapTimeStr[8];
    snprintf(lapTimeStr, sizeof(lapTimeStr), "%07lu", fastestLapTime);

    // Copy into positions 7–8 and 9–15 (0-based index)
    memcpy(&fileName[6], lapNumStr, 2);  // Best Lap Number: pos 7–8, (6 in 0-based index), copy lapNumStr to fileName at position 6
    memcpy(&fileName[8], lapTimeStr, 7); // Best Lap Time: pos 9–15, (8 in 0-based index)

    fram1.writeEnable(true);
    fram1.write(HEADER_ADDRESS, (uint8_t *)&header, sizeof(header)); // Update the header
    fram1.writeEnable(false);

    return true;
}

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
    // Serial.print("Crossed Line: ");
    // Serial.println(crossed ? "Yes" : "No");

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
            //    Serial.println("Ignoring first crossing.");
            ignoreFirstLap = false;   // Clear the flag after the first crossing
            wasOverLine = true;       // Set to true to avoid immediate re-triggering
            lastLapStart = timestamp; // Reset the last lap start time
            currentLap = 1;           // Set current lap to 1
            // Serial.println("Current lap set to:");
            // Serial.println(currentLap);
            return false;
        }
        // Debug: Print debounce check
        // Serial.print("Debounce Check: Time since last lap = ");
        // Serial.println(timestamp - lastLapStart);

        if (timestamp - lastLapStart >= debounceTime)
        {
            lastLapTime = timestamp - lastLapStart;
            lastLapStart = timestamp;
            wasOverLine = true;

            lapsCompleted++;
            currentLap++;

            if (lastLapTime < fastestLapTime)
            {
                fastestLapTime = lastLapTime;     // Update fastest lap time
                fastestLapNumber = lapsCompleted; // Update fastest lap number
                updateFramFilenameIfNewBest(header.fileName, sizeof(header.fileName));
            }

            // Debug: Print lap details
            // Serial.println("Lap crossed!");
            // Serial.print("Last lap time: ");
            // Serial.println(lastLapTime);
            // Serial.print("Total laps completed: ");
            // Serial.println(lapsCompleted);
            // Serial.print("Best Lap time: ");
            // Serial.println(fastestLapTime);
            // Serial.print("Best Lap number: ");
            // Serial.println(fastestLapNumber);
            // Serial.print("Current Timestamp: ");
            // Serial.println(timestamp);
            // Serial.print("Current lap number: ");
            // Serial.println(currentLap);

            // Serial.println();

            wasOverLine = false; // Reset after processing the lap
            return true;
        }
        else
        {
            // Debug: Print debounce failure
            //   Serial.println("Debounce time not met. Lap not triggered.");
        }
    }

    wasOverLine = crossed; // Update state based on crossing

    // Debug: Print updated wasOverLine state
    // Serial.print("Updated wasOverLine: ");
    // Serial.println(wasOverLine ? "True" : "False");

    return false;
}

uint8_t LapTimer::getCurrentLapNumber()
{
    return currentLap;
}
