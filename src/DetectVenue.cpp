#include "DetectVenue.h"

// Function to calculate the distance using the Haversine formula
double haversine(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6371000; // Earth's radius in meters
    double dLat = (lat2 - lat1) * (M_PI / 180.0);
    double dLon = (lon2 - lon1) * (M_PI / 180.0);

    lat1 = lat1 * (M_PI / 180.0);
    lat2 = lat2 * (M_PI / 180.0);

    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1) * cos(lat2) *
                   sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return R * c; // Distance in meters
}

String detectVenue(float currentLatitude, float currentLongitude)
{

    // Cadwell startline coords ish.. 53.310269, -0.059400
    // float cadwel_lat = 53.310269;
    // float cadwel_lng = -0.059400;
    // float perimeter = 0.1000;

    const double tolerance = 150.0;

    // const double targetLatitude = 53.722850;
    // const double targetLongitude = -2.467709; // Home coords ish fudged

    const double targetLatitude = 53.738324;
    const double targetLongitude = -2.473829; // work coords

    delay(1000); // time for gps to stabilize for venue lock

    // Calculate distance to the target coordinates
    double distance = haversine(currentLatitude, currentLongitude, targetLatitude, targetLongitude);

    // Check if within the tolerance
    if (distance <= tolerance)   // 0-9 or abc.... etc...
    {
        // Trigger lap completion logic
        Serial.println("Work...");
        return "1";
    }
    else
    {
        Serial.println("Not at the finish line.");
        return "0";
    }
}