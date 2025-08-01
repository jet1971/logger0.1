#include "DetectVenue.h"

double startLineLat;
double startLineLng;
double startLineLat2;
double startLineLng2;

struct Venue
{
    const char *name;
    double latitude;
    double longitude;
    double latitude2;
    double longitude2;
    double radius; // detection tolerance in meters
    char code;     // character to return, 1 for Work, 2 for Home, etc.
};

// known venues here
Venue knownVenues[] = {
    {"Work", 53.738728, -2.473479, 53.738616, -2.473487, 500.0, '1'}, // startLineLat, startLineLng, startLineLat2, startLineLng2, radius, code
    {"Home", 53.723057, -2.465386, 53.722933, -2.465327, 500.0, '2'},
    {"Aintree", 53.475249, -2.939913, 53.475332, -2.939922, 500.0, '3'},
    {"Services", 53.714574, -2.475667, 53.714570, -2.475778, 500.0, '4'},
    {"Cadwell", 53.310928, -0.061267, 53.310928, -0.061267, 500.0, '5'},
    {"Oulton", 53.180626, -2.613581, 53.180626, -2.613581, 500.0, '6'},
};

const int NUM_VENUES = sizeof(knownVenues) / sizeof(knownVenues[0]);

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
    delay(1000); // Allow GPS to stabilize

    for (int i = 0; i < NUM_VENUES; i++)
    {
        double distance = haversine(
            currentLatitude,
            currentLongitude,
            knownVenues[i].latitude,
            knownVenues[i].longitude);

        if (distance <= knownVenues[i].radius)
        {
            Serial.print("Detected venue: ");
            Serial.println(knownVenues[i].name);
            startLineLat = knownVenues[i].latitude;
            startLineLng = knownVenues[i].longitude;
            startLineLat2 = knownVenues[i].latitude2;
            startLineLng2 = knownVenues[i].longitude2;

            return String(knownVenues[i].code);
        }
    }

    return "0"; // No venue match
}
