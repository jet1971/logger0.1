#include "Settings.h"
#include <Preferences.h>

Preferences preferences; // Create a Preferences instance
                    

String loggerId = "000";     // Default Logger ID, must be exactly 3 characters
bool daylightSaving = false; // Default daylight saving time setting
int rpmMultiplier = 1;       // Default RPM multiplier setting
int tpsMin = 0;
int tpsMax = 20000;

void loadLoggerSettings()
{    
    preferences.begin("settings", true); // Open preferences for logger settings, true = read only
    String storedLoggerId = preferences.getString("loggerId", "VAN"); // Default to "VAN" if not set
    if (storedLoggerId.length() == 3)                                 // Check if the stored logger ID is exactly 3 characters
    {
        loggerId = storedLoggerId; // Use the stored logger ID
    }
    daylightSaving = preferences.getBool("daylightSaving", false);
    rpmMultiplier   = preferences.getInt("rpmMultiplier", 1); // Default to 1 if not set
    tpsMin = preferences.getInt("tpsMin", 0);
    tpsMax = preferences.getInt("tpsMax", 20000);
    preferences.end();

    Serial.print("Loading from preferences: ");
    Serial.println(loggerId);
    Serial.println(daylightSaving);
    Serial.println(rpmMultiplier);
    Serial.println(tpsMin);
    Serial.println(tpsMax);

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
