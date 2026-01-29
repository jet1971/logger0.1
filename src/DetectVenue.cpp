#include "DetectVenue.h"
#include <ArduinoJson.h>
#include <LittleFS.h>

double startLineLat;
double startLineLng;
double startLineLat2;
double startLineLng2;


Venue loadedVenues[20]; // Adjust size if needed
int loadedCount = 0;

void loadVenuesFromJson()
{
    File file = LittleFS.open("/venues.json", "r");
    if (!file)
    {
        Serial.println("Failed to open venues.json for reading");
        return;
    }

    JsonDocument doc; // Use a size that fits your JSON
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error)
    {
        Serial.print("JSON deserialization failed: ");
        Serial.println(error.c_str());
        return;
    }

    JsonArray arr = doc.as<JsonArray>();
    loadedCount = 0;

    for (JsonObject obj : arr)
    {
        if (loadedCount >= 20)
            break;

        loadedVenues[loadedCount].name = obj["name"].as<String>();
        loadedVenues[loadedCount].lat1 = obj["lat1"];
        loadedVenues[loadedCount].lng1 = obj["lng1"];
        loadedVenues[loadedCount].lat2 = obj["lat2"];
        loadedVenues[loadedCount].lng2 = obj["lng2"];
        loadedVenues[loadedCount].radius = obj["radius"];
        loadedVenues[loadedCount].code = obj["code"].as<String>()[0]; // first char

        loadedCount++;
    }

    Serial.printf("Loaded %d venues\n", loadedCount);

    Serial.print("🔎 Loaded from LittleFS: ");
    serializeJsonPretty(doc, Serial);                  // Uncomment to print full JSON
    Serial.println();
}

void saveVenuesToJson()
{
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();

    Serial.printf("🔍 Saving %d venues...\n", loadedCount);

    for (int i = 0; i < loadedCount; i++)
    {
        JsonObject obj = arr.createNestedObject();
        obj["name"] = loadedVenues[i].name;
        obj["lat1"] = loadedVenues[i].lat1;
        obj["lng1"] = loadedVenues[i].lng1;
        obj["lat2"] = loadedVenues[i].lat2;
        obj["lng2"] = loadedVenues[i].lng2;
        obj["radius"] = loadedVenues[i].radius;
        obj["code"] = String(loadedVenues[i].code);
    }

    File file = LittleFS.open("/venues.json", "w");
    if (!file)
    {
        Serial.println("❌ Failed to open venues.json for writing");
        return;
    }

    if (serializeJson(doc, file) == 0)
    {
        Serial.println("❌ Failed to write JSON to file");
    }
    file.close();

    Serial.println("✅ Venues saved to /venues.json");

    Serial.print("🔎 Final JSON: ");
    serializeJsonPretty(doc, Serial);
    Serial.println();

    // ✅ Immediately reload and print what was just saved
    // Serial.println("📂 Re-loading venues from saved file...");
    // loadVenuesFromJson(); // This prints venue list again
}

// known venues here
// Venue knownVenues[] = {
//     {"Work", 53.738728, -2.473479, 53.738616, -2.473487, 500.0, '1'}, // startLineLat, startLineLng, startLineLat2, startLineLng2, radius, code
//     {"Home", 53.723057, -2.465386, 53.722933, -2.465327, 500.0, '2'},
//     {"Aintree", 53.475249, -2.939913, 53.475332, -2.939922, 500.0, '3'},
//     {"Services", 53.714574, -2.475667, 53.714570, -2.475778, 500.0, '4'},
//     {"Cadwell", 53.310928, -0.061267, 53.310928, -0.061267, 500.0, '5'},
//     {"Oulton", 53.180626, -2.613581, 53.180626, -2.613581, 500.0, '6'},
//     {"IOM", 54.168267, -4.477173, 54.168389, -4.477313, 500.0, '7'},
//     // copilot generated
//     {"Brands Hatch", 51.338889, 0.281389, 51.338889, 0.281389, 500.0, '8'},
//     {"Snetterton", 52.501111, 0.936111, 52.501111, 0.936111, 500.0, '9'},
//     {"Three Sisters", 53.507324, -2.633878, 53.507393, -2.633911, 500.0, 'a'},
//     {"Portimão", 37.232098, -8.630921, 37.232142, -8.630734, 500.0, 'b'},
//     {"Unknown Venue", 0.0, 0.0, 0.0, 0.0, 500.0, '0'} // Default case
// };

// const int NUM_VENUES = sizeof(knownVenues) / sizeof(knownVenues[0]);

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
    loadVenuesFromJson();

    delay(1); // Allow GPS to stabilize, was 1000, AI told me to remove

    for (int i = 0; i < loadedCount; i++)
    {
        double distance = haversine(
            currentLatitude,
            currentLongitude,
            loadedVenues[i].lat1,
            loadedVenues[i].lng1);

        if (distance <= loadedVenues[i].radius)
        {
            Serial.print("Detected venue: ");
            Serial.println(loadedVenues[i].name);
            startLineLat = loadedVenues[i].lat1;
            startLineLng = loadedVenues[i].lng1;
            startLineLat2 = loadedVenues[i].lat2;
            startLineLng2 = loadedVenues[i].lng2;

            return String(loadedVenues[i].code);
        }
    }

    return "0"; // No venue match
}
