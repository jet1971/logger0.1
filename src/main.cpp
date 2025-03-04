#include <Arduino.h>
#include <NimBLEDevice.h>
#include <LittleFS.h> // Use the built-in LittleFS library
#include "LittleFSHandler.h"
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include "Adafruit_FRAM_SPI.h"
#include <Wire.h>

uint8_t FRAM_CS = 5;
uint8_t FRAM_SCK = 18;
uint8_t FRAM_MISO = 19;
uint8_t FRAM_MOSI = 23;

Adafruit_FRAM_SPI fram =
    Adafruit_FRAM_SPI(FRAM_SCK, FRAM_MISO, FRAM_MOSI, FRAM_CS);

uint8_t addrSizeInBytes = 3; // Default to address size three bytes

// TaskHandle_t Task1;

#define CHUNK_SIZE 500 // 512 causes corrupted data, 500 seems to work
#define BUTTON_PIN 12  // start ble server if true
#define START_LOGGING_PIN 15
#define LED 22
const uint8_t tachoInput = 13;

bool isBleServerStarted = false; // Flag to check if BLE server is already started

LittleFSHandler fsHandler; // Create an instance of the handler
String gpsCoords;

static NimBLEServer *pServer;
static NimBLECharacteristic *pReadFileCharacteristic;
static NimBLECharacteristic *pDeleteFileCharacteristic;
bool isSending = false;
bool fileNameSet = false;
File file;
char buffer[512];//was 480
int bytesRead = 0;
bool acknowledgmentReceived = true;
size_t currentFilePosition = 0; // Track the current position in the file
size_t fileSize = 0;
bool makeTimeStamp = true; // currently used to identify Log files
TinyGPSPlus gps;

String formattedDay;
String formattedmonth;
String formattedHour;
String formattedMin;

//  stuff -------------------------------------------------------------------------

// String logId; // becomes the log name, set in setup, re-enable later

bool loggerStarted = false;

// Header at FRAM address 0
struct FramLogHeader
{
    char fileName[36];
    u_int32_t dataLength;
};

struct LogRecord
{
    uint32_t timestamp;
    float lat;
    float lng;
    uint16_t mph; // speed x 10
    uint16_t rpm;
    uint8_t tps;                // throttle position sensor
    uint8_t afr;                // lambda x 10
    uint8_t iPressure;          // intake pressure use MPX5100DP, 0-14.5PSI OR 0 TO 100KPA Tranmit in KPA
    uint8_t airTemperature;     // air temp
    uint8_t coolantTemperature; // coolant temp
    uint8_t oilPressure;        //  oil pressure
    uint8_t bVoltage;           // battery voltage x 10
    uint8_t spare;              // spare
};

static const size_t HEADER_ADDRESS = 0x00;
static const uint32_t DATA_START = sizeof(FramLogHeader);

FramLogHeader header;
FramLogHeader header2;
LogRecord record;

// FastLogEntry fastLogEntry;   // Create a FastLogEntry struct
// SlowLogEntry slowLogEntry;   // Create a SlowLogEntry struct

uint32_t currentDataAddress = DATA_START;

// buffer stuff -------------------------------------------------------------------------

String logId = "";

// rpm stuff -------------------------------------------------------------------------
unsigned long rpm = 0;
hw_timer_t *timer0 = NULL;             // Declare a pointer to a hardware timer object
unsigned long minValidPulseLength = 1; // Adjust as needed (in millis)
unsigned long lastPulseTime;
unsigned long rotation = 0;
int rpmMultiplier = 4;          // temp hard coded
int DEFAULT_RPM_MULTIPLIER = 2; // not used yet in logger app, set ig app gui eventually

unsigned long startLowPulseTimer;
unsigned long startHighPulseTimer;
bool flag = false;
bool flag2 = false;
bool validLowPulse = false;
bool validHighPulse = false;
unsigned long duration;
unsigned long previousLowPulseTimeStamp = 0;

// rpm stuff --------------------------------------------------------------------------

// logging times-----------------------------------------------------------------------
unsigned long lastTemperatureLogTime = 0; // For tracking temperature logging
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
unsigned long lastTempTime2 = 0;

unsigned long lastGpsLogTime = 0; // For tracking GPS logging
unsigned long lastRpmLogTime = 0; // For tracking RPM logging
const unsigned long slowLogginginterval = 1250;
const unsigned long fastLoggingInterval = 100;
const unsigned long tempLogInterval2 = 100;

const unsigned long gpsLogInterval = 100;
const unsigned long rpmLogInterval = 100;

//--------------------------------------------------------------------------------------------------------
const char UBLOX_INIT[] PROGMEM = {
    // Disable NMEA
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24, // GxGGA off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, // GxGLL off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, // GxGSA off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, // GxGSV off
                                                                                                    // 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, // GxVTG off

    // Disable UBX
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDC, // NAV-PVT off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, // NAV-POSLLH off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0, // NAV-STATUS off

    // Enable UBX
    //  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
    //  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
    //  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on

    // Rate
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12, //(10Hz)
    // 0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
    // 0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)
};

//-----------------------------------------------------------------------------------------------------
//                                  tacho stuff
//-----------------------------------------------------------------------------------------------------
void IRAM_ATTR
pulseChange()

{
    unsigned long currentMicros = micros();

    // Debounce the input signal
    if (currentMicros - lastPulseTime > minValidPulseLength)
    {
        rotation++;                    // Increment pulse count
        lastPulseTime = currentMicros; // Update last pulse time
    }
}

void IRAM_ATTR rpmTimer()
{
    rpm = (rotation * 60) * rpmMultiplier; // eg rpmMultiplier = 2,  means 1 pulse per rotation, Steelie, wasted spark
    rotation = 0;                          //

} // something with a cam sensor will be 1 pulse every 2 rotations, so multiplier will be 4

//-----------------------------------------------------------------------------

void sendFileDetails(NimBLECharacteristic *pCharacteristic)
{
    String fileListJson = fsHandler.listFiles();

    const size_t chunkSize = 500; // MTU size is it? or just the chunk size? seem to work set at 500, was 512 = corrupted data..
    size_t length = fileListJson.length();
    Serial.print("Complete length of Json doc: ");
    Serial.println(length);

    for (size_t i = 0; i < length; i += chunkSize)
    {
        String chunk = fileListJson.substring(i, i + chunkSize); // Get the next chunk
                                                                 // String chunk = fileListJson; // Get the next chunk
        Serial.println(chunk);
        pCharacteristic->setValue(chunk); // chunk, chunk.c_str()
        pCharacteristic->notify();
        // Serial.print("This is the Json string: ");
        // Serial.println(chunk);
        delay(10); // Small delay to allow the client to process the notification
    }
    // After sending all the chunks, send the "END" marker
    delay(10);
    pCharacteristic->setValue("END");
    pCharacteristic->notify(); // Notify the client that transfer is complete
    Serial.println("Finished sending JSON file list. Sent END signal.");
}

void sendChunk(NimBLECharacteristic *pCharacteristic, File file)
{
    if (!file || file.available() == 0)
    {
        Serial.println("No file or empty! Sending END");
        pCharacteristic->setValue("END");
        pCharacteristic->notify(); // Indicate end of data transfer
        file.close();              // Close the file after sending the END signal
        isSending = false;         // Stop sending after file transfer is done
        fileNameSet = false;       // reset file name bool thing

        return;
    }

    int bytesRead = file.readBytes(buffer, CHUNK_SIZE); // Read the next chunk of data

    if (bytesRead > 0)
    {
        Serial.print("Sending chunk of size: ");
        Serial.println(bytesRead);
        pCharacteristic->setValue((uint8_t *)buffer, bytesRead); // Send chunk
        pCharacteristic->notify();                               // Notify client
    }

    // Optionally, you can track the total bytes sent
    static int totalBytesSent = 0;
    totalBytesSent += bytesRead;
    Serial.print("Total bytes sent so far: ");
    Serial.println(totalBytesSent);
}

class ServerCallbacks : public NimBLEServerCallbacks
{
    void onConnect(NimBLEServer *pServer)
    {
        Serial.println("Client connected");
        NimBLEDevice::startAdvertising();
    };

    void onDisconnect(NimBLEServer *pServer)
    {
        Serial.println("Client disconnected - start advertising");
        NimBLEDevice::startAdvertising();
    };
};

//------------------------------------------------------------------------------------------

class DownloadFileCallback : public NimBLECharacteristicCallbacks
{
    std::string fileName = "";

    void onWrite(NimBLECharacteristic *pCharacteristic)

    {
        if (!fileNameSet)
        {
            fileName = pCharacteristic->getValue();
            fileName = "/" + fileName;
            fileNameSet = true;
            Serial.println("This is the file name sent from the app " + String(fileName.c_str()));
        }

        //   Serial.println("onWrite called");
        //   Serial.print("Client wrote to characteristic UUID: ");
        //   Serial.println(pCharacteristic->getUUID().toString().c_str());

        std::string uuidStr = pCharacteristic->getUUID().toString();
        uuidStr.erase(0, uuidStr.find_first_not_of(" \t\n\r\f\v"));
        uuidStr.erase(uuidStr.find_last_not_of(" \t\n\r\f\v") + 1);

        // Serial.print("Trimmed UUID: ");
        // Serial.println(uuidStr.c_str());

        if (uuidStr == "a00d" || uuidStr == "0xa00d" || uuidStr == "0000a00d-0000-1000-8000-00805f9b34fb")
        {
            //   Serial.println("Correct characteristic UUID");
            std::string value = pCharacteristic->getValue();
            Serial.print("Received value: ");
            Serial.println(value.c_str());

            if (value == "SEND_FILE_DETAILS")

            {
                sendFileDetails(pCharacteristic);
            }

            if (value == "GET_LARGE_FILE" && !isSending)
            {
                Serial.println("GET_LARGE_FILE request received");

                file = fsHandler.openFile(fileName.c_str()); // Open the file with name received above

                if (!file)
                {
                    Serial.println("Failed to open file after GET_LARGE_FILE command recognised");
                    return;
                }

                //   Serial.println("File opened successfully. Starting transfer...");
                isSending = true;
                sendChunk(pCharacteristic, file); // Start sending the first chunk
            }
            else if (value == "NEXT_CHUNK" && isSending)
            {
                // Send the next chunk of the file
                //    Serial.println("NEXT_CHUNK request received");
                sendChunk(pCharacteristic, file);
            }
        }
    }
};

//------------------------------------------------------------------------------------------

class DeleteFileCallback : public NimBLECharacteristicCallbacks
{
    void onWrite(NimBLECharacteristic *pCharacteristic)
    {
        std::string fileName = pCharacteristic->getValue();
        fileName = "/" + fileName;

        if (LittleFS.exists(fileName.c_str()))
        {
            LittleFS.remove(fileName.c_str());
            Serial.println("File deleted: " + String(fileName.c_str()));
        }
        else
        {
            Serial.println("File not found: " + String(fileName.c_str()));
        }
    }
};

//------------------------------------------------------------------------------------------

class ReadFileCallback : public NimBLECharacteristicCallbacks
{
    void onWrite(NimBLECharacteristic *pCharacteristic)
    {

        std::string uuidStr = pCharacteristic->getUUID().toString();
        uuidStr.erase(0, uuidStr.find_first_not_of(" \t\n\r\f\v"));
        uuidStr.erase(uuidStr.find_last_not_of(" \t\n\r\f\v") + 1);

        if (uuidStr == "f00d" || uuidStr == "0xf00d" || uuidStr == "0000f00d-0000-1000-8000-00805f9b34fb")
        {
            //   Serial.println("Correct characteristic UUID");
            std::string value = pCharacteristic->getValue();
            Serial.print("Received value: ");
            Serial.println(value.c_str());

            if (value == "SEND_FILE_DETAILS")
           
            {
                sendFileDetails(pCharacteristic);
            }
        }
    }
};

//-----------------------------------------------------------------------------------------------

void startBLEServer()
{
    Serial.println("Starting NimBLE Server");

    NimBLEDevice::init("JT DataLogger_1");

    NimBLEDevice::setMTU(512);

    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService *pBaadService = pServer->createService("BAAD");

    //-----------------------------------------------------------------------------------------------

    NimBLECharacteristic *pReadFileCharacteristic = pBaadService->createCharacteristic(
        "F00D",
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    pReadFileCharacteristic->setCallbacks(new ReadFileCallback());

    //-----------------------------------------------------------------------------------------------

    NimBLECharacteristic *pDeleteFileCharacteristic = pBaadService->createCharacteristic(
        "D00D",
        NIMBLE_PROPERTY::WRITE);

    pDeleteFileCharacteristic->setCallbacks(new DeleteFileCallback());
    //-----------------------------------------------------------------------------------------------

    NimBLECharacteristic *pDownloadFileCharacteristic = pBaadService->createCharacteristic(
        "A00D",
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    pDownloadFileCharacteristic->setCallbacks(new DownloadFileCallback());

    //-----------------------------------------------------------------------------------------------

    pBaadService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(pBaadService->getUUID());
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    Serial.println("Advertising Started");
}
//--------------------------------------------------------------------------------------

bool checkServerStartCondition()
{
    // Check if the button is pressed
    if (digitalRead(BUTTON_PIN) == HIGH)
    {
        Serial.println("Button pressed, starting BLE server...");
        return true;
    }
    return false;
}
//----------------------------------------------------------------------------------------

// Function to calculate the distance using the Haversine formula
double haversine(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6371000; // Earth's radius in meters
    double dLat = (lat2 - lat1) * (M_PI / 180.0);
    double dLon = (lon2 - lon1) * (M_PI / 180.0);

    lat1 = lat1 * (M_PI / 180.0);
    lat2 = lat2 * (M_PI / 180.0);

    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1) * cos(lat2) *
                   sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return R * c; // Distance in meters
}

String detectVenue(float currentLatitude, float currentLongitude)
{

    // Cadwell startline coords ish.. 53.310269, -0.059400
    // float cadwel_lat = 53.310269;
    // float cadwel_lng = -0.059400;
    // float perimeter = 0.1000;

    const double tolerance = 75.0;

    // const double targetLatitude = 53.722850;
    // const double targetLongitude = -2.467709; // Home coords ish fudged

    const double targetLatitude = 53.738324;
    const double targetLongitude = -2.473829; // work coords

    delay(1000); // time for gps to stabilize for venue lock

    // Calculate distance to the target coordinates
    double distance = haversine(currentLatitude, currentLongitude, targetLatitude, targetLongitude);

    // Check if within the tolerance
    if (distance <= tolerance)
    {
        // Trigger lap completion logic
        Serial.println("Work...");
        return "wo";
    }
    else
    {
        Serial.println("Not at the finish line.");
        return "na";
    }
}

void setFramHeader(const char *name)
{
    // Fill header with initial values

    strncpy(header.fileName, name, sizeof(header.fileName) - 1); // copy "name" to header.filename, the sizeof bit tells the function how many characters to copy? maybe
    header.fileName[sizeof(header.fileName) - 1] = '\0';         // Ensure null-termination
    header.dataLength = 0;                                       // Start with zero data length

    fram.writeEnable(true);
    fram.write(HEADER_ADDRESS, (uint8_t *)&header, sizeof(header)); // starts at position 0, writes the header to FRAM
    fram.writeEnable(false);

    Serial.println("Filename set, Logging started");
    Serial.println(header.fileName);
    Serial.println(sizeof(header));

    fram.read(HEADER_ADDRESS, (uint8_t *)&header2, sizeof(header2)); // Read the header from FRAM);
    Serial.println("This is the file name from fram");
    Serial.println(header2.fileName);
}

void flushRingToFram()
{
    fram.writeEnable(true);
    fram.write(currentDataAddress, (uint8_t *)&record, sizeof(record));
    fram.writeEnable(false);
    currentDataAddress += sizeof(record);
    header.dataLength += sizeof(record);

    fram.writeEnable(true);
    fram.write(HEADER_ADDRESS, (uint8_t *)&header, sizeof(header)); // Update the header with the new data length
    fram.writeEnable(false);
}

// void Task1code(void *parameter) //------------------- core 1 ---------------------------------
// {
//     String *logBuffer = static_cast<String *>(parameter); // Cast the parameter to String*
//     for (;;)
//     {
//         if (logBuffer->length() >= bufferSize)
//         {
//             flushBuffer(reinterpret_cast<const uint8_t *>(logBuffer->c_str()), logBuffer->length());
//         }
//     }
// }

//------------------ core 1 ----------------------------------------------------------------------

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
        Serial.print(readRecord.mph/10.0);
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

void setup()
{

    Serial.begin(115200);
    Serial2.begin(9600); // TX = GPIO 17 RX = 16

    //---------------------------------------- stuff----------------------------------------------------------------

    // xTaskCreatePinnedToCore(
    //     Task1code, /* Function to implement the task */
    //     "Task1",   /* Name of the task */
    //     4096,      /* Stack size in words */
    //    &logBuffer,      /* Task input parameter */
    //     1,         /* Priority of the task */
    //     &Task1,    /* Task handle. */
    //     1          /* Core where the task should run */
    // );

    //---------------------------------------- stuff----------------------------------------------------------------

    //---------------------------------Timer/Tahco ----------------------------------------------------------------

    // /* Use 1st timer of 4,  the 0 is the first timer
    //   /* 1 tick takes 1/(80MHZ/80) = 1us so set divider to 80

    //---------------------------------------- stuff----------------------------------------------------------------

    timer0 = timerBegin(0, 80, true);              // Timer 0, prescaler of 80, count up
    timerAttachInterrupt(timer0, &rpmTimer, true); // Attach rpmTimer function to the timer
    timerAlarmWrite(timer0, 250000, true);         // Set alarm to call rpmTimer function every quarter second
    timerAlarmEnable(timer0);                      // Enable the timer alarm

    attachInterrupt(tachoInput, pulseChange, CHANGE); // Attach interrupt to the sensor pin

    //--------------------------------------------------------------------------------------------------------

    // send configuration data in UBX protocol
    for (int i = 0; i < sizeof(UBLOX_INIT); i++)
    {
        Serial2.write(pgm_read_byte(UBLOX_INIT + i));
        delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
    }
    //-----------------------------------------------------------------------
    if (!fsHandler.begin())
    {
        Serial.println("Failed to initialize LittleFS");
        if (LittleFS.format())
        {
            Serial.println("Format successful. File system mounted.");
        }
        else
        {
            Serial.println("Format failed.");
        }
        return;
    }
    //-----------------------------------------------------------------------
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    if (fram.begin())
    {
        Serial.println("Found SPI FRAM");
    }
    else
    {
        Serial.println("No SPI FRAM found ... check your connections\r\n");
        while (1)
            ;
    }
    fram.setAddressSize(addrSizeInBytes); // Set the address size for the FRAM chip, mucho importante!!

    fram.read(HEADER_ADDRESS, (uint8_t *)&header2, sizeof(header2)); // Read the header from FRAM
    Serial.println("This is the file name about to be saved to LittleFS");
    Serial.println(header2.fileName);
    Serial.println("Data length");
    Serial.println(header2.dataLength);

    if (!LittleFS.exists(header2.fileName))
    {
        static const size_t BUFFER_SIZE = 512;
        uint8_t buffer2[BUFFER_SIZE]; // allocated on the stack
        uint32_t address = DATA_START;

        // 3) Loop in chunks until we've written all data
        size_t bytesLeft = header2.dataLength;
        while (bytesLeft > 0)
        {
            // Decide how many bytes to read this iteration
            size_t toRead = (bytesLeft < BUFFER_SIZE) ? bytesLeft : BUFFER_SIZE;

            // Read that chunk from FRAM
            fram.read(address, buffer2, toRead);

            // Write the chunk to the file
            fsHandler.writeData(header2.fileName, buffer2, toRead);

            // Update pointers/counters
            address += toRead;
            bytesLeft -= toRead;
        }

        file.close();
        Serial.print("Wrote ");
        Serial.print(header2.dataLength);
        Serial.print(" bytes to ");
        Serial.println(header2.fileName);
    }
    else
    {
        Serial.println("File already exists so no need to write to LittleFS");
    }
    String tempFileList = fsHandler.listFiles();
    Serial.println(tempFileList);
    delay(1000);

    //-----------------------------------------------------------------------
    // Paths for the old and new filenames
    // String oldFilePath = "/2203202422:21.txt";
    // String newFilePath = "/na2203202422:21.txt";

    // Serial.println("here sir!");

    // // Check if the old file exists before renaming
    // if (LittleFS.exists(oldFilePath))
    // {
    //     // Attempt to rename the file
    //     if (LittleFS.rename(oldFilePath, newFilePath))
    //     {
    //         Serial.println("File renamed successfully");
    //     }
    //     else
    //     {
    //         Serial.println("Failed to rename file");
    //     }
    // }
    // else
    // {
    //     Serial.println("Old file does not exist");
    // }
    // na1410202420:30.txt

    //-----------------------------------------------------------------------
    String filePath = "/na0103202522:15.txt"; // File to delete
    // Check if the file exists before trying to remove it
    if (LittleFS.exists(filePath))
    {
        // Remove the file
        if (LittleFS.remove(filePath))
        {
            Serial.println("File deleted successfully");
        }
        else
        {
            Serial.println("Failed to delete file");
        }
    }
    else
    {
        Serial.println("File does not exist");
    }
    // Optionally, unmount LittleFS when done
    //  LittleFS.end();
    //-----------------------------------------------------------------------
    // Write data to a file
    // if (fsHandler.writeData("/data.txt", "Hello, World!"))
    // {
    //     Serial.println("Data written to /data.txt");
    // }

    // Read data from a file
    // String fileData = fsHandler.readData("/ho2601202509:00.txt");
    // Serial.println("Little FS File Data: " + fileData);

    // fileSize = fsHandler.getFileSize("/data2.txt"); // get the file size

    // fsHandler.listFiles(); // List all files in LittleFS
    //-----------------------------------------------------------------------

    // Initialize the button pin
    pinMode(START_LOGGING_PIN, INPUT_PULLDOWN);
    pinMode(BUTTON_PIN, INPUT_PULLDOWN);
    pinMode(tachoInput, INPUT_PULLUP);
    pinMode(LED, OUTPUT);
    //  digitalWrite(LED, HIGH);

    // Initial check in setup to see if BLE server should start right away
    if (checkServerStartCondition())
    {
        startBLEServer();
        isBleServerStarted = true; // Set flag to indicate BLE server has started
    }
    else
    {
        Serial.println("Condition not met, waiting to start BLE server...");
    }
    Serial.print("sizeof(LogRecord) = ");
    Serial.println(sizeof(LogRecord));

    //  used to check the offset of each struct element

    //   Serial.print("offset of timestamp = ");
    //    Serial.print(offsetof(LogRecord, timestamp));
    //     Serial.print("offset of lat = ");
    //     Serial.println(offsetof(LogRecord, lat));
    //     Serial.print("offset of lng = ");
    //     Serial.println(offsetof(LogRecord, lng));
    //     Serial.print("offset of mph = ");
    //     Serial.println(offsetof(LogRecord, mph));
    //     Serial.print("offset of rpm = ");
    //     Serial.println(offsetof(LogRecord, rpm));
    //     Serial.print("offset of tps = ");
    //     Serial.println(offsetof(LogRecord, tps));
    //     Serial.print("offset of afr = ");
    //     Serial.println(offsetof(LogRecord, afr));
    //     Serial.print("offset of iPressure = ");
    //     Serial.println(offsetof(LogRecord, iPressure));
    //     Serial.print("offset of airTemperature = ");
    //     Serial.println(offsetof(LogRecord, airTemperature));
    //     Serial.print("offset of coolantTemperature = ");
    //     Serial.println(offsetof(LogRecord, coolantTemperature));
    //     Serial.print("offset of oilPressure = ");
    //     Serial.println(offsetof(LogRecord, oilPressure));
    //     Serial.print("offset of bVoltage = ");
    //     Serial.println(offsetof(LogRecord, bVoltage));
    //     Serial.print("offset of spare = ");
    //     Serial.println(offsetof(LogRecord, spare));

//    readAllStructs("/na0103202522:15.txt");
}

void loop()
{

    //   Serial.print("loop running on core ");
    //   Serial.println(xPortGetCoreID());

    // loop and ble runs running on core 1

    // Only check and start BLE server if it's not already started
    if (!isBleServerStarted)
    {
        if (checkServerStartCondition())
        {
            startBLEServer();          // Start BLE server
            isBleServerStarted = true; // Set the flag to prevent re-calling the function
        }
    }

    //---------------------------------------------------------------------------------------------

    while (Serial2.available() > 0)
    {

        double currentLat = gps.location.lat();
        double currentLng = gps.location.lng();
        float mph = gps.speed.mph();
        u_int16_t mphInt = mph * 10;
    

        gps.encode(Serial2.read());

        if ((gps.location.isUpdated()) & (makeTimeStamp == true))
        {

            String venue = detectVenue(gps.location.lat(), gps.location.lng());

            int day = (gps.date.day());
            int month = (gps.date.month());
            int year = (gps.date.year());
            int hour = (gps.time.hour());
            int min = (gps.time.minute());
            makeTimeStamp = false;

            if (day < 10)
            {
                String modDay = String(day);
                formattedDay = "0" + modDay; // should add a zero if day less than 10, eg: returns 02 instead of 2,
            }
            else
            {
                formattedDay = day;
            }

            if (month < 10)
            {
                String modmonth = String(month);
                formattedmonth = "0" + modmonth;
            }
            else
            {
                formattedmonth = month;
            }
            if (hour < 10)
            {
                String modHour = String(hour);
                formattedHour = "0" + modHour;
            }
            else
            {
                formattedHour = hour;
            }
            if (min < 10)
            {
                String modMin = String(min);
                formattedMin = "0" + modMin;
            }
            else
            {
                formattedMin = min;
            }

            logId = "/" + String(venue) + String(formattedDay) + String(formattedmonth) + String(year) + String(formattedHour) + ":" + String(formattedMin) + ".txt"; // assemble a file name from the gps time
            Serial.print("First print of logID = ");
            Serial.println(logId);
        }

        //---------------------------------------------------------------------------------------------

        // if (!loggerStarted && (gps.location.isUpdated()) && (millis() % 1000 == 0))
        if (!loggerStarted && (gps.location.isUpdated()))
        {
            {
                Serial.println("Logging started");
                loggerStarted = true; // Set the flag to prevent re-calling the function
                setFramHeader(logId.c_str());
                Serial.println(logId);
                digitalWrite(LED, HIGH);
            }
        }
        //---------------------------------------------------------------------------------------------
        if (loggerStarted)
        {
            if (millis() - previousMillis2 >= fastLoggingInterval)
            {
                previousMillis2 = millis();

                record.timestamp = millis();
                record.lat = currentLat;
                record.lng = currentLng;
                record.mph = mphInt;
                record.rpm = rpm;
                record.tps = 45;
                record.afr = 132;      // 13.2
                record.iPressure = 21; // probably kpa 0-100
                record.airTemperature = 23;
                record.coolantTemperature = 24;
                record.oilPressure = 0; // 0
                record.bVoltage = 125;  // 12.5
                record.spare = 0;

                flushRingToFram();
            }
        }
    }

    //-------------------------- TEST STUFF --------------------------------------------------------

    // if (!loggerStarted && (digitalRead(START_LOGGING_PIN) == HIGH) && (millis() % 1000 == 0))
    // {
    //     Serial.println("Logging Button Pressed");
    //     loggerStarted = true; // Set the flag to prevent re-calling the function
    //     setFramHeader(logId.c_str());
    // }

    // if (loggerStarted)
    // {
    //     if (millis() - previousMillis2 >= fastLoggingInterval)
    //     {
    //         previousMillis2 = millis();
    //         record.timestamp = previousMillis2;
    //         record.lat = 53.310269;
    //         record.lng = -2.460925;
    //         record.afr = 251;
    //         record.rpm = 12500;
    //         record.iPressure = 9;
    //         record.mph = 123;
    //         record.tps = 45;
    //         record.airTemperature = 23;
    //         record.coolantTemperature = 24;
    //         record.bVoltage = 12.5;

    //         flushRingToFram();
    //     }
    // }
    //Serial.println(rpm);
}
