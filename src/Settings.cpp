#include "Settings.h"
#include <Preferences.h>

Preferences preferences; // Create a Preferences instance
                    

String loggerId = "000";     // Default Logger ID, must be exactly 3 characters
bool daylightSaving = false; // Default daylight saving time setting
int rpmMultiplier = 1;       // Default RPM multiplier setting
int tpsMin = 0;
int tpsMax = 20000;
int venueListVersion = 0;

void loadLoggerSettings()
{    
    preferences.begin("settings", true); // Open preferences for logger settings, true = read only
    String storedLoggerId = preferences.getString("loggerId", "***"); // Default to "***" if not set
    if (storedLoggerId.length() == 3)                                 // Check if the stored logger ID is exactly 3 characters
    {
        loggerId = storedLoggerId; // Use the stored logger ID
    }
    daylightSaving = preferences.getBool("daylightSaving", false);
    rpmMultiplier = preferences.getInt("rpmMultiplier", 1); // Default to 1 if not set
    tpsMin = preferences.getInt("tpsMin", 0);
    tpsMax = preferences.getInt("tpsMax", 20000);
    venueListVersion = preferences.getInt("venueVersion", 0);
    preferences.end();

   // saveVenueListVersion(); // Uncomment this line and comment out venueListVersion line above to reset version.

    Serial.println("Loading from preferences: ");
    Serial.print("Logger ID: ");
    Serial.println(loggerId);
    Serial.print("Daylight Saving: ");
    Serial.println(daylightSaving);
    Serial.print("RPM Multiplier: ");
    Serial.println(rpmMultiplier);
    Serial.print("Min Raw TPS: ");
    Serial.println(tpsMin);
    Serial.print("Max Raw TPS: ");
    Serial.println(tpsMax);
    Serial.print("Venue List Version Number: ");
    Serial.println(venueListVersion);

}

void saveLoggerSettings()
{
    preferences.begin("settings", false); // write mode
    preferences.putString("loggerId", loggerId);
    preferences.putInt("tpsMin", tpsMin);
    preferences.putInt("tpsMax", tpsMax);
    preferences.putBool("daylightSaving", daylightSaving);
    preferences.putInt("rpmMultiplier", rpmMultiplier);
    preferences.end();

    Serial.print("Saved: Logger ID: ");
    Serial.println(loggerId);
    Serial.print("Daylight Saving: ");
    Serial.println(daylightSaving ? "Enabled" : "Disabled");
    Serial.print("RPM Multiplier: ");
    Serial.println(rpmMultiplier);
    Serial.print("Min Raw TPS: ");
    Serial.println(tpsMin);
    Serial.print("Max Raw TPS: ");
    Serial.println(tpsMax);

    loadLoggerSettings();
}

void saveVenueListVersion()// This is just the number of the list, sent to app to check if update needed
{
    preferences.begin("settings", false); // write mode
    preferences.putInt("venueVersion", venueListVersion);
    preferences.end();
    Serial.print("Venue List Version Number: ");
    Serial.println(venueListVersion);
}