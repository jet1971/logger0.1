#ifndef LOGGERTASK_H
#define LOGGERTASK_H

#include <Arduino.h>
//#include <TinyGPSPlus.h>
#include "LogRecord.h"

// Declare global variables (defined in .cpp or main.cpp)
extern int RED_LED;
extern int GREEN_LED;
extern int LAMBDA_CONTROL_PIN;
extern int SEND_RPM_ENABLE_PIN;

extern bool loggerStarted;
extern bool makeTimeStamp;

extern String logFilename;
extern String formattedDay;
extern String formattedmonth;
extern String formattedHour;
extern String formattedMin;

extern String version;
extern String fastestLapNumber;
extern String fastestLapTime;


extern LogRecord record;

extern const uint32_t FRAM_SAFETY; // keep 64 bytes free
extern const uint32_t FRAM_USABLE_END;

extern const uint32_t FRAM_SIZE;
extern uint32_t currentDataAddress;
extern const uint32_t DATA_START;
extern const size_t HEADER_ADDRESS;

extern float cachedCoolantTemp;



// Function declaration
void loggerTask(void *parameter);

#endif

