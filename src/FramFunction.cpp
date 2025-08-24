#include "FramFunctions.h"

#include "LoggerTask.h"
#include "FramLogHeader.h"
#include "FRAMModule.h"

bool framFull = false;
portMUX_TYPE framMux = portMUX_INITIALIZER_UNLOCKED;

// wrap every FRAM read/write
void safeWrite(Adafruit_FRAM_SPI &fram, uint32_t addr, const uint8_t *buf, size_t len)
{
    taskENTER_CRITICAL(&framMux);
    fram.writeEnable(true);
    fram.write(addr, buf, len);
    fram.writeEnable(false);
    taskEXIT_CRITICAL(&framMux);
}

void safeRead(Adafruit_FRAM_SPI &fram, uint32_t addr, uint8_t *buf, size_t len)
{
    taskENTER_CRITICAL(&framMux);
    fram.read(addr, buf, len);
    taskEXIT_CRITICAL(&framMux);
}

inline uint32_t framRemaining()
{
    if (currentDataAddress >= FRAM_USABLE_END)
        return 0;
    return FRAM_USABLE_END - currentDataAddress;
}

void setFramHeader(const char *name)
{
    memset(&header, 0, sizeof(header));
    strncpy(header.fileName, name, sizeof(header.fileName) - 1);
    header.dataLength = 0;

    currentDataAddress = DATA_START;
    framFull = false;

    safeWrite(fram1, HEADER_ADDRESS, (uint8_t *)&header, sizeof(header));
}

void flushRingToFram()
{
    if (framFull)
        return;

    // guard against accidental pointer drift
    if (currentDataAddress < DATA_START)
        currentDataAddress = DATA_START;

    if (framRemaining() < sizeof(record))
    {
        framFull = true;
        // Persist the *final* header once, and stop touching FRAM afterward
     //   safeWrite(fram1, HEADER_ADDRESS, (uint8_t *)&header, sizeof(header));
        Serial.println("FRAM FULL - logging stopped cleanly");
        return;
    }

    // write one record
    safeWrite(fram1, currentDataAddress, (uint8_t *)&record, sizeof(record));

    currentDataAddress += sizeof(record);
    header.dataLength += sizeof(record);

    // update header so dataLength always matches last complete record
    safeWrite(fram1, HEADER_ADDRESS, (uint8_t *)&header, sizeof(header));
}




// void setFramHeader(const char *name)
// {
//     memset(&header, 0, sizeof(header));
//     strncpy(header.fileName, name, sizeof(header.fileName) - 1);
//     header.fileName[sizeof(header.fileName) - 1] = '\0';
//     header.dataLength = 0;

//     currentDataAddress = DATA_START;
//     framFull = false;

//     fram1.writeEnable(true);
//     fram1.write(HEADER_ADDRESS, (uint8_t *)&header, sizeof(header));
//     fram1.writeEnable(false);
// }


// inline uint32_t framRemaining()
// {
//     if (currentDataAddress >= FRAM_USABLE_END)
//         return 0;
//     return FRAM_USABLE_END - currentDataAddress;
// }

// void flushRingToFram()
// {
//     if (framFull)
//     {
//         return; // already out of space; do nothing
//     }

//     // Check there is enough space for one whole record
//     if (framRemaining() < sizeof(record))
//     {
//         // Mark full, persist header once, signal if you want, then stop
//         framFull = true;

//         // (Optional) set a flag in header to indicate "closed normally"
//         // header.flags |= 0x01; // if you have such a field

//         fram1.writeEnable(true);
//         fram1.write(HEADER_ADDRESS, (uint8_t *)&header, sizeof(header));
//         fram1.writeEnable(false);

//         // (Optional) LED or log
//         // digitalWrite(RED_LED, HIGH);
//          Serial.println("FRAM FULL - logging stopped cleanly");
//         return;
//     }

//     // Write one record
//     fram1.writeEnable(true);
//     fram1.write(currentDataAddress, (uint8_t *)&record, sizeof(record));
//     fram1.writeEnable(false);

//     currentDataAddress += sizeof(record);
//     header.dataLength += sizeof(record);

//     // Update header so dataLength always matches bytes safely written
//     fram1.writeEnable(true);
//     fram1.write(HEADER_ADDRESS, (uint8_t *)&header, sizeof(header));
//     fram1.writeEnable(false);
// }

// void setFramHeader(const char *name)
// {
//     memset(&header, 0, sizeof(header));// Clear the header structure before setting values

//     strncpy(header.fileName, name, sizeof(header.fileName) - 1); // copy "name" to header.filename, the sizeof bit tells the function how many characters to copy? maybe
//     header.fileName[sizeof(header.fileName) - 1] = '\0';         // Ensure null-termination
//     header.dataLength = 0;                                       // Start with zero data length

//     fram1.writeEnable(true);
//     fram1.write(HEADER_ADDRESS, (uint8_t *)&header, sizeof(header)); // starts at position 0, writes the header to FRAM
//     fram1.writeEnable(false);


//     Serial.println("Filename set, Logging started");
//     Serial.println(header.fileName);
//     Serial.println(sizeof(header));

//     fram1.read(HEADER_ADDRESS, (uint8_t *)&header2, sizeof(header2)); // Read the header from FRAM);
//     Serial.println("This is the file name from fram");
//     Serial.println(header2.fileName);
//  }

//  void flushRingToFram()
//  {

//      fram1.writeEnable(true);
//      fram1.write(currentDataAddress, (uint8_t *)&record, sizeof(record));
//      fram1.writeEnable(false);
//      currentDataAddress += sizeof(record);
//      header.dataLength += sizeof(record);
//      fram1.writeEnable(true);
//      fram1.write(HEADER_ADDRESS, (uint8_t *)&header, sizeof(header)); // Update the header with the new data length
//      fram1.writeEnable(false);
//  }

//portMUX_TYPE framMux = portMUX_INITIALIZER_UNLOCKED;
// const uint32_t DATA_START = sizeof(FramLogHeader); // 36 in your case
// extern uint32_t currentDataAddress = DATA_START;

// void setFramHeader(const char *name)
// {
//     taskENTER_CRITICAL(&framMux); // protect FRAM ops (see #2)
//     memset(&header, 0, sizeof(header));
//     strncpy(header.fileName, name, sizeof(header.fileName) - 1);
//     header.fileName[sizeof(header.fileName) - 1] = '\0';
//     header.dataLength = 0;

//     currentDataAddress = DATA_START; // <<< IMPORTANT

//     fram1.writeEnable(true);
//     fram1.write(HEADER_ADDRESS, (uint8_t *)&header, sizeof(header));
//     fram1.writeEnable(false);

//     Serial.println("Filename set, Logging started");
//     Serial.println(header.fileName);

//     // optional: read back while still locked
//     fram1.read(HEADER_ADDRESS, (uint8_t *)&header2, sizeof(header2));
//     taskEXIT_CRITICAL(&framMux);

//     Serial.println("Filename read back from FRAM:");
//     Serial.println(header2.fileName);
// }

// void flushRingToFram()
// {
//     size_t bytesLeft = sizeof(record);
//     const uint8_t *src = reinterpret_cast<const uint8_t *>(&record);

//     while (bytesLeft > 0)
//     {
//         Adafruit_FRAM_SPI *activeFram;
//         uint32_t localAddress;

//         if (currentDataAddress < FRAM_SIZE)
//         {
//             activeFram = &fram1;
//             localAddress = currentDataAddress;
//         }
//         else
//         {
//             activeFram = &fram2;
//             localAddress = currentDataAddress - FRAM_SIZE;
//         }

//         size_t maxHere = FRAM_SIZE - localAddress;
//         size_t chunk = (bytesLeft < maxHere) ? bytesLeft : maxHere;

//         activeFram->writeEnable(true);
//         activeFram->write(localAddress, src, chunk);
//         activeFram->writeEnable(false);

//         currentDataAddress += chunk;
//         src += chunk;
//         bytesLeft -= chunk;
//     }

//     header.dataLength += sizeof(record);

//     // persist header (fixed location on fram1)
//     fram1.writeEnable(true);
//     fram1.write(HEADER_ADDRESS, (uint8_t *)&header, sizeof(header));
//     fram1.writeEnable(false);
// }

// void flushRingToFram()
// {
//     Adafruit_FRAM_SPI *activeFram;
//     uint32_t localAddress;

//     if (currentDataAddress < FRAM_SIZE)
//     {
//         activeFram = &fram1;
//         localAddress = currentDataAddress;
//     }
//     else
//     {
//         activeFram = &fram2;
//         localAddress = currentDataAddress - FRAM_SIZE;
//     }

//     activeFram->writeEnable(true);
//     activeFram->write(localAddress, (uint8_t *)&record, sizeof(record));
//     activeFram->writeEnable(false);

//     currentDataAddress += sizeof(record);
//     header.dataLength += sizeof(record);

//     // Always store the header at the same location (e.g., in fram1)
//     fram1.writeEnable(true);
//     fram1.write(HEADER_ADDRESS, (uint8_t *)&header, sizeof(header));
//     fram1.writeEnable(false);
// }

