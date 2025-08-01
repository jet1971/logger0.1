#include <Arduino.h>
#include <NimBLEDevice.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include <Wire.h>
#include <nvs_flash.h> // Include the header for NVS functions
#include <ArduinoJson.h>
#include <Ticker.h>

#include "ADS1115Module.h"
#include <LittleFS.h>
#include "LittleFSHandler.h"
#include "StructSizeOffset.h"
#include "LogRecord.h"
#include "UartTask.h"
#include "LapTimer.h"
#include "DetectVenue.h"
#include "FRAMModule.h"
#include "Settings.h"
#include "TemperatureFunction.h"
#include "AirTemperatureFunction.h"
#include "LoggerTask.h"
#include "FramFunctions.h"
#include "FramLogHeader.h"
#include "MainShared.h"
#include "GPSTask.h"
#include "Tacho.h"


Ticker liveDataTicker; // used for sending live data
bool liveDataStreaming = false;

#define CHUNK_SIZE 500 // 512 causes corrupted data, 500 seems to work

bool bleActive = true;

// const uint8_t tachoInput = 4; // Tacho input pin
#define TXD1 27 // Custom UART TX pin
#define RXD1 13 // Custom UART RX pin

// #define TXD1 13 // Custom UART TX pin
// #define RXD1 27 // Custom UART RX pin

bool isBleServerStarted = false; // Flag to check if BLE server is already started

LittleFSHandler fsHandler; // Create an instance of the handler
String gpsCoords;

// Preferences preferences; // Preferences instance for storing logger settings

static NimBLEServer *pServer;
static NimBLECharacteristic *pReadFileCharacteristic;
static NimBLECharacteristic *pDeleteFileCharacteristic;
NimBLECharacteristic *pLiveDataCharacteristic = nullptr;

bool isSending = false;
bool fileNameSet = false;
File file;
char buffer[512]; // was 480
int bytesRead = 0;
bool acknowledgmentReceived = true;
size_t currentFilePosition = 0; // Track the current position in the file
size_t fileSize = 0;

// logging times-----------------------------------------------------------------------
// unsigned long lastTemperatureLogTime = 0; // For tracking temperature logging
// unsigned long previousMillis1 = 0;
// unsigned long previousMillis2 = 0;
// unsigned long lastTempTime2 = 0;

// unsigned long lastGpsLogTime = 0; // For tracking GPS logging
// unsigned long lastRpmLogTime = 0; // For tracking RPM logging
// const unsigned long slowLogginginterval = 1250;

// const unsigned long tempLogInterval2 = 100;

// const unsigned long gpsLogInterval = 100;
// const unsigned long rpmLogInterval = 100;

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
void sendEngineTemeperatureCalibration(NimBLECharacteristic *pCharacteristic)
{
    JsonDocument doc;
    doc["V0"] = voltagePoints[0];
    doc["V1"] = voltagePoints[1];
    doc["V2"] = voltagePoints[2];
    doc["V3"] = voltagePoints[3];
    doc["V4"] = voltagePoints[4];
    doc["V5"] = voltagePoints[5];
    doc["V6"] = voltagePoints[6];
    // Serialize the entire JSON object
    String jsonString;
    serializeJson(doc, jsonString);
    pCharacteristic->setValue(jsonString);
    pCharacteristic->notify();
}

void sendLoggerSettings(NimBLECharacteristic *pCharacteristic)
{
    JsonDocument doc;
    doc["LoggerId"] = loggerId;
    doc["daylightSaving"] = daylightSaving; // send these values back so they can be seen in UI
    doc["rpmMultiplier"] = rpmMultiplier;   // NOT SURE CHECK APP IF IT USES RETURNED VALUE   //RPM FUNCTION HIDDING IN UART TASK!!
    doc["tpsMin"] = tpsMin;                 // DON'T NEED TO RETURN THIS VALUE??
    doc["tpsMax"] = tpsMax;                 // DON'T NEED TO RETURN THIS VALUE??
    // Serialize the entire JSON object
    String jsonString;
    serializeJson(doc, jsonString);
    pCharacteristic->setValue(jsonString);
    pCharacteristic->notify();
}

class LoggerSettingsCallback : public NimBLECharacteristicCallbacks
{
    void onWrite(NimBLECharacteristic *pCharacteristic) override
    {

        std::string value = pCharacteristic->getValue();

        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, value);

        if (error)
        {
            Serial.println("❌ JSON parse error");
            return;
        }

        const char *cmd = doc["cmd"];
        if (!cmd)
        {
            Serial.println("❌ No 'cmd' field");
            return;
        }

        if (strcmp(cmd, "GET_SETTINGS") == 0) // SEND THE CURRECT SETTINGS BACK TO GUI
        {
            sendLoggerSettings(pCharacteristic);
        }

        else if (strcmp(cmd, "SAVE_SETTINGS") == 0) // INCOMING DATA TO BE SET IN MEMORY AND SAVED TO PREFERENCES
        {
            loggerId = doc["loggerId"].as<String>();
            if (loggerId.length() != 3)
            {
                Serial.println("Logger ID must be exactly 3 characters long.");
                return;
            }

            daylightSaving = doc["daylightSaving"].as<bool>();
            rpmMultiplier = doc["rpmMultiplier"].as<int>(); // RPM FUNCTION HIDDING IN UART TASK!!
            tpsMin = doc["minRawTPS"].as<int>();
            tpsMax = doc["maxRawTPS"].as<int>();
            saveLoggerSettings(); // Write data to preferences (in settings.cpp)
        }
        else if (strcmp(cmd, "GET_E_T_C") == 0) // ENGINE TEMPERATURE CALIBRATION, SEND BACK TO GUI
        {
            sendEngineTemeperatureCalibration(pCharacteristic);
        }
        else if (strcmp(cmd, "SAVE_E_T_C") == 0)
        {
            voltagePoints[0] = doc["V0"].as<float>(); // Put these values in memory
            voltagePoints[1] = doc["V1"].as<float>();
            voltagePoints[2] = doc["V2"].as<float>();
            voltagePoints[3] = doc["V3"].as<float>();
            voltagePoints[4] = doc["V4"].as<float>();
            voltagePoints[5] = doc["V5"].as<float>();
            voltagePoints[6] = doc["V6"].as<float>();
            saveEngineTemperatureCalibration(); // Write data to preferences (in TemperatureFuntion.cpp)
        }
        else
        {
            Serial.print("⚠️ Unknown cmd: ");
            Serial.println(cmd);
        }
    }
};

void sendLiveData()
{

    // Serial.println("⚡ Starting live data stream");

    if (pLiveDataCharacteristic == nullptr)
    {
        Serial.println("❌ Live data characteristic is NULL!");
        return;
    }

    JsonDocument doc;
    doc["tps"] = (readTps(ADS1, 3));
    doc["rpm"] = rpm;
    doc["rawTps"] = (ADS1.readADC(3));
    doc["bVolts"] = (readSensorFiltered(ADS2, 0, 51900.0, 10000.0)); // FUDGED VALUE (51900, RESISTOR IS 49900)??
    doc["lambda"] = (zeitronixLambda(ADS1, 1, 3300.0, 6800.0));      // USING SPARE 1
    // doc["lambda"] = (mapLambda(ADS2, 0, 3300.0, 6800.0));
    doc["engineTemp"] = (interpolateEngineTemperature(ADS2, 3, 100000.0, 100000.0)); // Engine temperature,  USING SPARE 2
    doc["engineTempVolts"] = (readAndCompensate(ADS2, 3, 100000.0, 100000.0));       // ENGINE TEMPERATURE SENSOR VOLTAGE
    doc["airTemp"] = (interpolateAirTemperature(ADS2, 2));

    String output;

    serializeJson(doc, output);
    // Serial.println(output);
    pLiveDataCharacteristic->setValue(output);
    pLiveDataCharacteristic->notify();
}

class LiveDataCallback : public NimBLECharacteristicCallbacks
{
    void onWrite(NimBLECharacteristic *pLiveDataCharacteristic)
    {
        std::string value = pLiveDataCharacteristic->getValue();

        if (value == "START")
        {
            Serial.println("⚡ Starting live data stream");
            liveDataStreaming = true;
            liveDataTicker.attach_ms(250, sendLiveData); // Send every 500 ms
        }
        else if (value == "STOP")
        {
            Serial.println("🛑 Stopping live data stream");
            liveDataStreaming = false;
            liveDataTicker.detach();
        }
    }
};

void checkSerial1and2Status()
{
    if (Serial1.available())
    {
        Serial.println("Serial1 is still active!");
    }
    else
    {
        Serial.println("Serial2 is OFF or no data available.");
    }
    if (Serial2.available())
    {
        Serial.println("Serial2 is still active!");
    }
    else
    {
        Serial.println("Serial2 is OFF or no data available.");
    }
}

//------------------------------------------------------------------------------------------

void stopSerial1and2ForBLE()
{
    Serial1.end();
    delay(200);
    Serial2.end(); // Turn off Serial2 to free CPU time                  // checkSerial1and2Status();
    delay(200);
}

//-----------------------------------------------------------------------------------------------

void startBLEServer()
{
    //  stopSerial1and2ForBLE(); // Stop Serial1 and Serial2 to free up CPU time
    //  Serial.println("Turning off Serial1 and Serial2 to free up CPU time");
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
    // Logger Settings
    NimBLECharacteristic *pSettingsCharacteristic = pBaadService->createCharacteristic(
        "B00D",
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    pSettingsCharacteristic->setValue(""); // Clear any old/stale data

    pSettingsCharacteristic->setCallbacks(new LoggerSettingsCallback());

    //-----------------------------------------------------------------------------------------------

    pLiveDataCharacteristic = pBaadService->createCharacteristic(
        "C00D", NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::WRITE);

    pLiveDataCharacteristic->setValue(""); // Clear any old/stale data

    pLiveDataCharacteristic->setCallbacks(new LiveDataCallback());

    //-----------------------------------------------------------------------------------------------

    pBaadService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(pBaadService->getUUID());
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    Serial.println("Advertising Started");
}
//----------------------------------------------------------------------------------------------

//=== STOP BLE ===
void stopBLEServer()
{
    Serial.println("Stopping BLE...");
    NimBLEDevice::deinit(true); // Frees RAM too
    bleActive = false;
}

//-----------------------------------------------------------------------------------------------

// bool checkServerStartCondition()
// {
//     // Check if the button is pressed
//     if (digitalRead(BUTTON_PIN) == HIGH)
//     {
//         Serial.println("Button pressed, starting BLE server...");
//         return true;
//     }
//     return false;
// }
//----------------------------------------------------------------------------------------

void setup()
{
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(LAMBDA_CONTROL_PIN, OUTPUT);
    // pinMode(SEND_RPM_ENABLE_PIN, OUTPUT);
    pinMode(TACHO_PIN, INPUT_PULLDOWN);
    digitalWrite(LAMBDA_CONTROL_PIN, HIGH); // LAMBDA HEATER OFF

    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1); // RXD1 = GPIO 27 TXD1 = GPIO 13, slave esp32
    Wire.begin();
    setupADS(); // SETUP ADC
    Serial2.begin(9600);
    // delay(1000);     // Wait for GPS to boot
    // Serial2.flush(); // Clear the buffer

    // Serial2.write("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"); // GGA + RMC
    // delay(200);
    // Serial2.write("$PMTK300,100,0,0,0,0*2C\r\n"); // Fix interval 100ms
    // delay(200);
    // Serial2.write("$PMTK220,100*2F\r\n"); // Output rate 100ms

    // Serial2.begin(115200); // TX = GPIO 17 RX = 16, GPS UNIT
    // // send configuration data in UBX protocol
    // Serial.println("Sending UBX commands to configure GPS...");
    for (int i = 0; i < sizeof(UBLOX_INIT); i++)
    {
        Serial2.write(pgm_read_byte(UBLOX_INIT + i));
        delayMicroseconds(5000);
    }
    // // //
    // // delay(2000);
    // // Serial2.println("$PMTK220,100*2F"); // 10Hrz update rate
    // // delay(2000);
    // // Serial2.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");

    // delay(1000);
    // Serial2.flush(); // Clear any remaining data

    // Serial2.write("$PMTK300,100,0,0,0,0*2C\r\n"); // Position fix rate every 100ms
    // delay(200);
    // Serial2.write("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"); // Set NMEA output
    // delay(200);
    // Serial2.write("$PMTK220,100*2F\r\n");                                   // Set update rate to 10Hz
    // delay(200);
    // // Serial2.println("$PMTK220,100*2F");

    Serial.println("UBX commands sent.");
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

    if (fram1.begin())
    {
        Serial.println("Found SPI FRAM");
    }
    else
    {
        Serial.println("No SPI FRAM found ... check your connections\r\n");
        while (1)
            ;
    }
    //-----------------------------------------------------------------------
    fram1.setAddressSize(addrSizeInBytes); // Set the address size for the FRAM chip, mucho importante!!

    //------------------------------------------------------------------------------------

    fram1.read(HEADER_ADDRESS, (uint8_t *)&header2, sizeof(header2)); // Read the header from FRAM
    Serial.println("This is the file name about to be saved to LittleFS");
    Serial.println(header2.fileName);

    Serial.println("Data length");
    Serial.println(header2.dataLength);

    if (header2.fileName[0] != '/')
    {
        Serial.println("Data does not start with /, so it is not a valid file name");
    }
    else if

        (!LittleFS.exists(header2.fileName))
    {
        // digitalWrite(RED_LED, HIGH);
        digitalWrite(RED_LED, !digitalRead(RED_LED));

        static const size_t BUFFER_SIZE = 512;
        uint8_t buffer2[BUFFER_SIZE]; // allocated on the stack
        uint32_t address = DATA_START;

        // 3) Loop in chunks until we've written all data
        size_t bytesLeft = header2.dataLength;
        while (bytesLeft > 0)
        {
            digitalWrite(RED_LED, !digitalRead(RED_LED));
            // Decide how many bytes to read this iteration
            size_t toRead = (bytesLeft < BUFFER_SIZE) ? bytesLeft : BUFFER_SIZE;

            // Read that chunk from FRAM
            fram1.read(address, buffer2, toRead);

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
        digitalWrite(RED_LED, LOW);
        //  ESP.restart(); // Restart the ESP32, so to make sure GPS gets setup commands again
    }
    else
    {
        Serial.println("File already exists so no need to write to LittleFS");
    }
    //------------------------------------------------------------------------------------
    String tempFileList = fsHandler.listFiles();
    Serial.println(tempFileList);
    delay(500);

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
    String filePath = "/1VAN1000000000123223072025.txt"; // File to delete

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
        Serial.println("File to delete does not exist");
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
    //    loggerId = fsHandler.readData("/loggerId.txt");
    //    Serial.println("ID read from Little FS: " + loggerId);

    // fileSize = fsHandler.getFileSize("/data2.txt"); // get the file size

    // fsHandler.listFiles(); // List all files in LittleFS
    //-----------------------------------------------------------------------

    //  fsHandler.formatLittleFS(); // WARNING! formats/erase all data from flash file system

    loadLoggerSettings();               // put logger settings in to memory
    loadEngineTemperatureCalibration(); // put temperature calibration settings in to memory
    loadAirTemperatureCalibration();
    startBLEServer();
    isBleServerStarted = true; // Set flag to indicate BLE server has started
    //----------------------------------------------------------------------------------------
    //    if (!preferences.begin("settings", false))
    //    {
    //        Serial.println("Failed to open preferences. Formatting NVS...");
    //        nvs_flash_erase(); // Erase the NVS partition
    //        nvs_flash_init();  // Reinitialize NVS
    //    }
    // preferences.end();

    digitalWrite(GREEN_LED, HIGH); // just showing the loop is running GREEN

    //----------------------------------------------------------------------------------------
    xTaskCreatePinnedToCore(
        loggerTask,   // Task function
        "LoggerTask", // Name
        8192,         // Stack size (adjust if needed)
        NULL,         // Parameters
        1,            // Priority
        NULL,         // Task handle (optional)
        0             // Core (0 or 1)
    );
    //----------------------------------------------------------------------------------------

    xTaskCreatePinnedToCore(
        gpsTask,
        "GpsTask",
        4096,
        NULL,
        2, // higher priority good, bad??
        NULL,
        1);
    //----------------------------------------------------------------------------------------
}

void loop()
{
    // digitalWrite(SEND_RPM_ENABLE_PIN, HIGH); // for debug and live data, ********** MAKE AUTO **********
    // uartTask();
    tacho();
    //  digitalWrite(LAMBDA_CONTROL_PIN, LOW); // LOW = SWITCHES HEATER CIRCUIT ON

    static float filtered_engTemp = 0.0; // Initialize with a default value
    filtered_engTemp = 0.1 * cachedCoolantTemp + 0.9 * filtered_engTemp;

    if (filtered_engTemp > 6) // 6.0 = 60 degrees
    {
        digitalWrite(LAMBDA_CONTROL_PIN, LOW); // LOW = SWITCHES HEATER CIRCUIT ON
    }


    // Serial.println(exp_engTemp);
    // Serial.println(LAMBDA_CONTROL_PIN);

    // Serial.print("AFR Voltage ");
    // Serial.print(doubleReadSensor(ADS1, 1, 3300.0, 6800.0)); // AFR VOLTAGE TEST... MEASUREING SPARE 1
    // Serial.print("  ");
    // Serial.println(mapLambda(ADS2, 0, 3300.0, 6800.0));
    //----------------------------------------------------------------------------------
    // Serial.println(ADS1.readADC(2));                                // read the raw value from the TPS ADC
    // Serial.println(readSensor(ADS1, 2, 3300, 6800.0));                       // TPS VOLTAGE
    // Serial.println(readTps(ADS1, 2));                                         // TPS VALUE
    //------------------------------------------------------------------------------------

    //  Serial.print("Battery: ");
    //  Serial.println(readSensor(ADS2, 1, 49900.0, 10000.0)); // BATT VOLTAGE TEST

    // Serial.print("Air Temperaure ");
    // Serial.println(interpolateAirTemperature(ADS2, 2));

    //------------------------------------------------------------------------------------------------
    // Read AFR voltage and calculate average
    //     double afrVoltage = readAndCompensate(ADS2, 0, 3300.0, 6800.0); // AFR voltage reading
    //     afrSum -= afrReadings[afrIndex];                                // Subtract the oldest reading from the sum
    //     afrReadings[afrIndex] = afrVoltage;                             // Store the new reading
    //     afrSum += afrVoltage;                                           // Add the new reading to the sum
    //     afrIndex = (afrIndex + 1) % numReadings;                        // Move to the next index (circular buffer)
    //     afrAverage = afrSum / numReadings;                              // Calculate the average
    //    // Print the average AFR voltage
    //   //  Serial.print("AFR Voltage (Average): ");

    //     float exp_afr = (mapLambda(ADS2, 0, 3300.0, 6800.0));
    //     exp_afr = 0.02 * exp_afr + (1.0 - 0.01) * exp_afr;
    //     Serial.print(exp_afr,0);

    //     Serial.print("      ");
    //     Serial.println(afrAverage, 2); // Print with 2 decimal places
    //-------------------------------------------------------------------------------------------------
    // digitalWrite(LED_1, HIGH);
    //    digitalWrite(LED_2, HIGH); // just showing the loop is running GREEN
    // digitalWrite(LED_1, HIGH); // just showing the loop is running RED

    // filteredValue = alpha * qsRawValue + (1.0 - alpha) * filteredValue;
    //  float exp_engTemp = 0.1 * (interpolateTemperature(ADS2, 3, 100000.0, 100000.0)) + (1.0 - 0.1) * exp_engTemp;
    //  //   //  Serial.println(exp_airTemp);
    //  if (exp_engTemp > 6) // 2.5 = 25 degrees
    //  {
    //      digitalWrite(LAMBDA_CONTROL_PIN, LOW); // LOW = SWITCHES HEATER CIRCUIT ON
    //  }

    // Serial.print(gps.date.day());
    // Serial.print("  ");
    // Serial.print(gps.date.month());
    // Serial.print("  ");
    // Serial.println(gps.date.year());

    //  Serial.print("Voltage ");
    //  Serial.println(readSensor(ADS1, 1, 3300.0, 6800.0)); // AFR VOLTAGE TEST... MEASUREING SPARE 1
    // Serial.println(interpolateTemperature(ADS1, 1, 3300.0, 6800.0)); // USING SPARE 2, NEED DIVIDER RESISTORS CHANGING DEPENDING ON MAX SENSOR VOLTAGE
        // 3600 if 5 volt max = 3.269 volts
    }
