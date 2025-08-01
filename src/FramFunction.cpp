#include "FramFunctions.h"

#include "LoggerTask.h"
#include "FramLogHeader.h"
#include "FRAMModule.h"

void setFramHeader(const char *name)
{
   
    strncpy(header.fileName, name, sizeof(header.fileName) - 1); // copy "name" to header.filename, the sizeof bit tells the function how many characters to copy? maybe
    header.fileName[sizeof(header.fileName) - 1] = '\0';         // Ensure null-termination
    header.dataLength = 0;                                       // Start with zero data length

    fram1.writeEnable(true);
    fram1.write(HEADER_ADDRESS, (uint8_t *)&header, sizeof(header)); // starts at position 0, writes the header to FRAM
    fram1.writeEnable(false);

    Serial.println("Filename set, Logging started");
    Serial.println(header.fileName);
    Serial.println(sizeof(header));

    fram1.read(HEADER_ADDRESS, (uint8_t *)&header2, sizeof(header2)); // Read the header from FRAM);
    Serial.println("This is the file name from fram");
    Serial.println(header2.fileName);
}

void flushRingToFram()
{

    fram1.writeEnable(true);
    fram1.write(currentDataAddress, (uint8_t *)&record, sizeof(record));
    fram1.writeEnable(false);
    currentDataAddress += sizeof(record);
    header.dataLength += sizeof(record);
    fram1.writeEnable(true);
    fram1.write(HEADER_ADDRESS, (uint8_t *)&header, sizeof(header)); // Update the header with the new data length
    fram1.writeEnable(false);
}