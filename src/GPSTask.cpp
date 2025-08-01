#include "GPSTask.h"

TinyGPSPlus gps;

// Define the variables
volatile double currentLat = 0.0, currentLng = 0.0, currentSpeed = 0.0;
volatile int day = 0, year = 0, month = 0, hour = 0, minute = 0;
volatile bool gpsValid = false;

void gpsTask(void *param)
{
    while (true)
    {
        while (Serial2.available())
        {
            gps.encode(Serial2.read());
        }
        if (gps.location.isUpdated())
        {
            currentLat = gps.location.lat();
            currentLng = gps.location.lng();
            currentSpeed = gps.speed.mph();
            day = gps.date.day();
            month = gps.date.month();
            year = gps.date.year();
            hour = gps.time.hour();
            minute = gps.time.minute();
            currentSpeed = gps.speed.mph();
            gpsValid = true;

            if (gps.location.isUpdated())
            {
                Serial.print("Time: ");
                Serial.println(millis());
            }
        }
        vTaskDelay(1);
    }
}
