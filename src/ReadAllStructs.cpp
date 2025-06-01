#include "ReadAllStructs.h"


LogRecord readRecord;

void readAllStructs(const char *filename)
{
    File file = LittleFS.open(filename, FILE_READ);
    if (!file)
    {
        Serial.println("Failed to open file for reading");
        return;
    }

    while (true)
    {
        // Attempt to read one struct
        size_t bytesRead = file.readBytes((char *)&readRecord, sizeof(readRecord));
        if (bytesRead < sizeof(readRecord))
        {
            // either EOF or partial read
            if (bytesRead > 0)
            {
                Serial.println("Warning: partial struct read, file may be truncated!");
            }
            break; // done reading
        }

        // Print or process the struct
        // Serial.print("=== Entry ===  ");
        Serial.print("timestamp: ");
        Serial.println(readRecord.timestamp);
        Serial.print(", lat: ");
        Serial.print(readRecord.lat, 6);
        Serial.print(", lng: ");
        Serial.print(readRecord.lng, 6);
        Serial.print(", mph: ");
        Serial.print(readRecord.mph / 10.0);
        Serial.print(", rpm: ");
        Serial.print(readRecord.rpm);
        Serial.print(", tps: ");
        Serial.println(readRecord.tps);
        Serial.print(", AFR: ");
        Serial.print(readRecord.afr);
        Serial.print(", iPressure: ");
        Serial.print(readRecord.iPressure);
        Serial.print(", airTemperature: ");
        Serial.print(readRecord.airTemperature);
        Serial.print(", coolantTemperature: ");
        Serial.print(readRecord.coolantTemperature);
        Serial.print(", oilPressure: ");
        Serial.print(readRecord.oilPressure);
        Serial.print(", bVoltage: ");
        Serial.println(readRecord.bVoltage);
        Serial.print(", spare: ");
        Serial.println(readRecord.spare);
        Serial.println();
    }

    file.close();
}