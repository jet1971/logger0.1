#include "StructSizeOffset.h"

void printStructSizeAndOffsets()
{
    Serial.println("Struct Size and Offsets:");
    Serial.println("-------------------------");

    Serial.print("sizeof(LogRecord) = ");
    Serial.println(sizeof(LogRecord));

    Serial.print("offset of timestamp = ");
    Serial.println(offsetof(LogRecord, timestamp));

    Serial.print("offset of lat = ");
    Serial.println(offsetof(LogRecord, lat));
    Serial.print("offset of lng = ");
    Serial.println(offsetof(LogRecord, lng));
    Serial.print("offset of mph = ");
    Serial.println(offsetof(LogRecord, mph));
    Serial.print("offset of rpm = ");
    Serial.println(offsetof(LogRecord, rpm));
    Serial.print("offset of tps = ");
    Serial.println(offsetof(LogRecord, tps));
    Serial.print("offset of afr = ");
    Serial.println(offsetof(LogRecord, afr));
    Serial.print("offset of iPressure = ");
    Serial.println(offsetof(LogRecord, iPressure));
    Serial.print("offset of airTemperature = ");
    Serial.println(offsetof(LogRecord, airTemperature));
    Serial.print("offset of coolantTemperature = ");
    Serial.println(offsetof(LogRecord, coolantTemperature));
    Serial.print("offset of oilPressure = ");
    Serial.println(offsetof(LogRecord, oilPressure));
    Serial.print("offset of bVoltage = ");
    Serial.println(offsetof(LogRecord, bVoltage));
    Serial.print("offset of spare = ");
    Serial.println(offsetof(LogRecord, spare));
}
//    readAllStructs("/na0103202522:15.txt");