// #ifndef FRAMMODULE_H
// #define FRAMMODULE_H

// #include "Adafruit_FRAM_SPI.h"

// // Define FRAM pins and objects
// extern Adafruit_FRAM_SPI fram;
// extern const size_t HEADER_ADDRESS;
// extern const uint32_t DATA_START;

// struct FramLogHeader
// {
//     char fileName[36];
//     uint32_t dataLength;
// };

// struct LogRecord
// {
//     uint32_t timestamp;
//     float lat;
//     float lng;
//     uint16_t mph;
//     uint16_t rpm;
//     uint8_t tps;
//     uint8_t afr;
//     uint8_t iPressure;
//     uint8_t airTemperature;
//     uint8_t coolantTemperature;
//     uint8_t oilPressure;
//     uint8_t bVoltage;
//     uint8_t spare;
// };

// bool initFRAM();
// void setFramHeader(const char *name);
// void flushRecordToFRAM(const LogRecord &record);

// #endif // FRAMMODULE_H
