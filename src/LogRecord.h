#ifndef LOGRECORD_H
#define LOGRECORD_H

#include <Arduino.h>

struct LogRecord
{
    uint32_t timestamp;
    float lat;
    float lng;
    uint16_t mph;
    uint16_t rpm;
    uint8_t tps;
    uint8_t afr;
    uint8_t iPressure;
    uint8_t airTemperature;
    uint8_t coolantTemperature;
    uint8_t oilPressure;
    uint8_t bVoltage;
    uint8_t spare;
};

#endif // LOGRECORD_H
