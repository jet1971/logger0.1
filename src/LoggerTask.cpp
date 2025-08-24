#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h" // Needed for critical section macros

#include "LoggerTask.h"
#include "DetectVenue.h"
#include "Settings.h"
#include "FramLogHeader.h"
#include "FRAMModule.h"
#include "FramFunctions.h"
#include "UartTask.h"
#include "ADS1115Module.h"
#include "AirTemperatureFunction.h"
#include "TemperatureFunction.h"
#include "LapTimer.h"
#include "MainShared.h"
#include "GPSTask.h"
#include "Tacho.h"

const unsigned long fastLoggingInterval = 100;

LapTimer *lapTimer = nullptr; // globally accessible

// LapTimer lapTimer(startLineLat, startLineLng, startLineLat2, startLineLng2, 6.0, 10000); // Work

// LapTimer lapTimer(53.738728, -2.473479, 53.738616, -2.473487, 6.0, 10000); // work

// LapTimer lapTimer(53.723057, -2.465386, 53.722933, -2.465327, 6.0, 10000);// Aster chase

// LapTimer lapTimer(53.714574, -2.475667, 53.714570, -2.475778, 6.0, 10000); // Services, left at roundabout up to next roundabout

// LapTimer lapTimer(53.475249, -2.939913, 53.475332, -2.939922, 6.0, 10000); //AINTREE

// === Global variable definitions (should only exist here ONCE) ===
int SPEED_THRESHOLD = 30; // mph

int RED_LED = 14;
int GREEN_LED = 2;
int LAMBDA_CONTROL_PIN = 25;
int SEND_RPM_ENABLE_PIN = 26;

bool loggerStarted = false;
bool makeTimeStamp = true;
unsigned long lastFastLogTime = 0; // Initialize lastFastLogTime

String logFilename = "";
String formattedDay;
String formattedmonth;
String formattedHour;
String formattedMin;

String version = "1";
String fastestLapNumber = "00";
String fastestLapTime = "0000000";

// TinyGPSPlus gps;
LogRecord record;

const uint32_t FRAM_SAFETY = 128; // keep 128 bytes free
const uint32_t FRAM_USABLE_END = FRAM_SIZE - FRAM_SAFETY;

const uint32_t FRAM_SIZE = 524288;
const uint32_t DATA_START = sizeof(FramLogHeader);
uint32_t currentDataAddress = DATA_START;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

float cachedCoolantTemp;

void loggerTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int skipEntries = 20;
    static double smoothedSpeed = 0.0;
    const double alpha = 0.9; // Smoothing factor for speed

    for (;;)
    {
        if (gpsValid && year > 2020 && makeTimeStamp)
        {
            String venue = detectVenue(currentLat, currentLng);

            // Create the lapTimer AFTER startLineLat etc. are set
            if (lapTimer == nullptr)
            {
                lapTimer = new LapTimer(startLineLat, startLineLng, startLineLat2, startLineLng2, 6.0, 10000);
                Serial.println("LapTimer initialized.");
                Serial.println("LapTimer startLineLat: " + String(startLineLat, 6));
                Serial.println("LapTimer startLineLng: " + String(startLineLng, 6));
                Serial.println("LapTimer startLineLat2: " + String(startLineLat2, 6));
                Serial.println("LapTimer startLineLng2: " + String(startLineLng2, 6));
            }


            int hourAdjusted = hour;
            if (!daylightSaving)
                hourAdjusted += 1;

            formattedDay = (day < 10) ? "0" + String(day) : String(day);
            formattedmonth = (month < 10) ? "0" + String(month) : String(month);
            formattedHour = (hourAdjusted < 10) ? "0" + String(hourAdjusted) : String(hourAdjusted);
            formattedMin = (minute < 10) ? "0" + String(minute) : String(minute);

            logFilename = "/" + venue + loggerId + version + fastestLapNumber + fastestLapTime +
                          formattedHour + formattedMin + formattedDay + formattedmonth + String(year) + ".txt";
            makeTimeStamp = false;
        }

        if (!loggerStarted && gpsValid && currentSpeed > SPEED_THRESHOLD)
        {
            loggerStarted = true;
            setFramHeader(logFilename.c_str());
            digitalWrite(RED_LED, HIGH);
            digitalWrite(GREEN_LED, LOW);
            digitalWrite(SEND_RPM_ENABLE_PIN, HIGH);
            stopBLEServer();
        }

        if (loggerStarted && gpsValid && millis() - lastFastLogTime >= fastLoggingInterval)
        {
            lastFastLogTime = millis();

            if (skipEntries > 0)
            {
                skipEntries--;
                Serial.print("Skip Entries Remaining: ");
                Serial.println(skipEntries);
            }
            else
            {
                double latCopy, lngCopy, speedCopy;
                bool valid;

                taskENTER_CRITICAL(&mux);
                latCopy = currentLat;
                lngCopy = currentLng;
                speedCopy = currentSpeed;
                valid = gpsValid;
                taskEXIT_CRITICAL(&mux);

                // Smooth speed data
                smoothedSpeed = alpha * speedCopy + (1.0 - alpha) * smoothedSpeed;

                record.timestamp = esp_timer_get_time() / 1000; // Use esp_timer_get_time() for precise timing
                record.lat = latCopy;
                record.lng = lngCopy;
                //  record.mph = smoothedSpeed; // Use smoothed speed
                record.mph = speedCopy * 10;
                record.rpm = rpm;
                record.tps = readTps(ADS1, 3);
                record.afr = zeitronixLambda(ADS1, 1, 3300.0, 6800.0);
               // record.afr = dynoJetLambda(ADS1, 1, 3300.0, 6800.0);
                record.iPressure = readSensor(ADS1, 0, 3300.0, 6800.0);
                record.airTemperature = interpolateAirTemperature(ADS2, 2);
                record.coolantTemperature = interpolateEngineTemperature(ADS2, 3, 100000.0, 100000.0);
                record.oilPressure = readSensor(ADS1, 2, 3300.0, 6800.0);
                record.bVoltage = readSensorFiltered(ADS2, 0, 51900.0, 10000.0);
                // record.spare = lapTimer.getCurrentLapNumber();
                record.spare = lapTimer->getCurrentLapNumber();

                cachedCoolantTemp = record.coolantTemperature; // Cache the coolant temperature for use in other tasks
                flushRingToFram();
            }
        }

        if (loggerStarted && gpsValid)
        {
            // if (lapTimer.checkLap(currentLat, currentLng, millis()))
            // {
            //     Serial.println("Lap detected!");
            // }
            if (lapTimer->checkLap(currentLat, currentLng, millis()))
            {
                Serial.println("Lap detected!");
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // Precise 10ms delay
    }
}
