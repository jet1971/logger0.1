// #include "FRAMModule.h"

// Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(18, 19, 23, 5); // SCK, MISO, MOSI, CS
// const size_t HEADER_ADDRESS = 0x00;
// const uint32_t DATA_START = sizeof(FramLogHeader);

// FramLogHeader header;
// uint32_t currentDataAddress = DATA_START;

// bool initFRAM()
// {
//     if (fram.begin())
//     {
//         fram.setAddressSize(3);
//         return true;
//     }
//     return false;
// }

// void setFramHeader(const char *name)
// {
//     strncpy(header.fileName, name, sizeof(header.fileName) - 1);
//     header.fileName[sizeof(header.fileName) - 1] = '\0';
//     header.dataLength = 0;
//     fram.writeEnable(true);
//     fram.write(HEADER_ADDRESS, (uint8_t *)&header, sizeof(header));
//     fram.writeEnable(false);
//     Serial.println("FRAM header set");
// }

// void flushRecordToFRAM(const LogRecord &record)
// {
//     fram.writeEnable(true);
//     fram.write(currentDataAddress, (uint8_t *)&record, sizeof(record));
//     fram.writeEnable(false);
//     currentDataAddress += sizeof(record);
//     header.dataLength += sizeof(record);
//     fram.writeEnable(true);
//     fram.write(HEADER_ADDRESS, (uint8_t *)&header, sizeof(header));
//     fram.writeEnable(false);
// }
