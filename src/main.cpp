#include <Arduino.h>
#include <NimBLEDevice.h>
#include <LittleFS.h> // Use the built-in LittleFS library
#include "LittleFSHandler.h"
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>

TaskHandle_t Task1;

#define CHUNK_SIZE 500 // Adjust based on your MTU can be 512 but might not be compatible on some crap
#define BUTTON_PIN 12  // start ble server if true
const uint8_t tachoInput = 13;

bool isBleServerStarted = false; // Flag to check if BLE server is already started

// String filePath = "/largefile.txt"; // File in LittleFS

LittleFSHandler fsHandler; // Create an instance of the handler
String gpsCoords;

static NimBLEServer *pServer;
static NimBLECharacteristic *pReadFileCharacteristic;
static NimBLECharacteristic *pDeleteFileCharacteristic;
bool isSending = false;
bool fileNameSet = false;
File file;
char buffer[CHUNK_SIZE];
int bytesRead = 0;
bool acknowledgmentReceived = true;
size_t currentFilePosition = 0; // Track the current position in the file
size_t fileSize = 0;
bool makeTimeStamp = true; // currently used to identify Log files
TinyGPSPlus gps;
String logId; // becomes the log name

String formattedDay;
String formattedmonth;
String formattedHour;
String formattedMin;

// buffer stuff -------------------------------------------------------------------------
String logBuffer = ""; // Buffer to hold GPS data

const int bufferSize = 512;     // Adjust buffer size as needed
const int flushInterval = 1000; // Flush buffer every second (1000 ms)
unsigned long lastFlushTime = 0;

// rpm stuff -------------------------------------------------------------------------
unsigned long rpm = 0;
hw_timer_t *timer0 = NULL;                // Declare a pointer to a hardware timer object
unsigned long minValidPulseLength = 1; // Adjust as needed (in millis)
unsigned long lastPulseTime;
unsigned long rotation = 0;
int rpmMultiplier = 1;          // temp hard coded
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
unsigned long lastTemperatureLogTime = 0;          // For tracking temperature logging
unsigned long lastGpsLogTime = 0;                  // For tracking GPS logging
unsigned long lastRpmLogTime = 0;                  // For tracking RPM logging
const unsigned long temperatureLogInterval = 1000; // 1 Hz = 1000 ms
const unsigned long gpsLogInterval = 100;          // 10 Hz = 100 ms
const unsigned long rpmLogInterval = 100;          // 1 Hz = 1000 ms

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
// void IRAM_ATTR
// pulseChange()

// {
//     unsigned long currentMicros = micros();

//     // Debounce the input signal
//     if (currentMicros - lastPulseTime > minValidPulseLength)
//     {
//         rotation++;                    // Increment pulse count
//         lastPulseTime = currentMicros; // Update last pulse time
//     }
// }

// void IRAM_ATTR rpmTimer()
// {
//     rpm = (rotation * 60) * rpmMultiplier; // eg rpmMultiplier = 2,  means 1 pulse per rotation, Steelie, wasted spark
//     rotation = 0;
//     //  Serial.println(rpm); //
// } // something with a cam sensor will be 1 pulse every 2 rotations, so multiplier will be 4

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
    // Serial.print("File size : ");
    // Serial.println(fileSize);

    // Serial.println(buffer);
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
            Serial.println("This is the file name sent " + String(fileName.c_str()));
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
        //   Serial.println("onWrite called");
        //   Serial.print("Client wrote to characteristic UUID: ");
        //   Serial.println(pCharacteristic->getUUID().toString().c_str());

        std::string uuidStr = pCharacteristic->getUUID().toString();
        uuidStr.erase(0, uuidStr.find_first_not_of(" \t\n\r\f\v"));
        uuidStr.erase(uuidStr.find_last_not_of(" \t\n\r\f\v") + 1);

        // Serial.print("Trimmed UUID: ");
        // Serial.println(uuidStr.c_str());

        if (uuidStr == "f00d" || uuidStr == "0xf00d" || uuidStr == "0000f00d-0000-1000-8000-00805f9b34fb")
        {
            //   Serial.println("Correct characteristic UUID");
            std::string value = pCharacteristic->getValue();
            Serial.print("Received value: ");
            Serial.println(value.c_str());

            if (value == "SEND_FILE_DETAILS")
            //   if (value == "GET_LARGE_FILE")
            {
                sendFileDetails(pCharacteristic);
            }

            // if (value == "GET_LARGE_FILE" && !isSending)
            // {
            //     Serial.println("GET_LARGE_FILE request received");
            //     file = fsHandler.openFile(fileName); // Open the file once

            //     if (!file)
            //     {
            //         Serial.println("Failed to open file after GET_LARGE_FILE command recognised");
            //         return;
            //     }

            //     //   Serial.println("File opened successfully. Starting transfer...");
            //     isSending = true;
            //     sendChunk(pCharacteristic, file); // Start sending the first chunk
            // }
            // else if (value == "NEXT_CHUNK" && isSending)
            // {
            //     // Send the next chunk of the file
            //     //    Serial.println("NEXT_CHUNK request received");
            //     sendChunk(pCharacteristic, file);
            // }
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

bool checkStartCondition()
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
        Serial.println("Work");
        return "wo";
    }
    else
    {
        Serial.println("Not at the finish line.");
        return "na";
    }
}

void flushBuffer(String data)
{

    if (fsHandler.appendData(logId.c_str(), data.c_str()))
    {
        //    Serial.println("GPS data written to " + logId);
        logBuffer = "";           // Clear the buffer
        lastFlushTime = millis(); // Update last flush time
    }
    // else
    // {
    //     Serial.println("Error opening file for writing");
    // }
}

void Task1code(void *parameter) //------------------- core 0 ---------------------------------
{

    for (;;)

     {
         duration = pulseIn(tachoInput, LOW);

         //     unsigned long currentMillis = millis();
         //     // Serial.print("Task1 running on core ");
         //     // Serial.println(xPortGetCoreID());

         //     if (digitalRead(tachoInput) == LOW && flag == false)
         //     {
         //         startLowPulseTimer = currentMillis;
         //         flag = true;
         //     }

         //     if (digitalRead(tachoInput) == LOW && (currentMillis - startLowPulseTimer >= minValidPulseLength))
         //     {
         //         validLowPulse = true;
         //     }

         //     if (digitalRead(tachoInput) == HIGH && flag2 == false)
         //     {
         //         startHighPulseTimer = millis();
         //         flag2 = true;
         //     }

         //     if (digitalRead(tachoInput) == HIGH && (millis() - startHighPulseTimer >= minValidPulseLength))
         //     {
         //         validHighPulse = true;
         //     }

         //     if (validLowPulse && validHighPulse)
         //     {

         //         // Calculate duration as the time between low pulse and high pulse
         //         duration = previousLowPulseTimeStamp - startLowPulseTimer;
         //         previousLowPulseTimeStamp = startLowPulseTimer;

         //         // Reset flags and valid pulse indicators
         //         validLowPulse = false;
         //         validHighPulse = false;
         //         flag = false;
         //         flag2 = false;

         //     }
          
     }
}
// rpm = 60000000 / startLowPulseTimer - startHighPulseTimer;
void setup()
{
    Serial.begin(115200);
    Serial2.begin(9600); // TX = GPIO 17 RX = 16

    // xTaskCreatePinnedToCore(
    //     Task1code, /* Function to implement the task */
    //     "Task1",   /* Name of the task */
    //     20000,     /* Stack size in words */
    //     NULL,      /* Task input parameter */
    //     0,         /* Priority of the task */
    //     &Task1,    /* Task handle. */
    //     0);        /* Core where the task should run */

    //---------------------------------Timer/Tahco ----------------------------------------------------------------

    // /* Use 1st timer of 4,  the 0 is the first timer
    //   /* 1 tick takes 1/(80MHZ/80) = 1us so set divider to 80

    //----------------------------------------tacho stuff----------------------------------------------------------------

    // timer0 = timerBegin(0, 80, true);              // Timer 0, prescaler of 80, count up
    // timerAttachInterrupt(timer0, &rpmTimer, true); // Attach rpmTimer function to the timer
    // timerAlarmWrite(timer0, 250000, true);         // Set alarm to call rpmTimer function every quarter second
    // timerAlarmEnable(timer0);                      // Enable the timer alarm

    // attachInterrupt(tachoInput, pulseChange, CHANGE); // Attach interrupt to the sensor pin

    //--------------------------------------------------------------------------------------------------------

    // send configuration data in UBX protocol
    for (int i = 0; i < sizeof(UBLOX_INIT); i++)
    {
        Serial2.write(pgm_read_byte(UBLOX_INIT + i));
        delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
    }

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
    String filePath = "/na0411202418:24.txt";

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
    // String fileData = fsHandler.readData("/22:10.txt");
    // Serial.println("File Data: " + fileData);

    // fileSize = fsHandler.getFileSize("/data2.txt"); // get the file size

    // fsHandler.listFiles(); // List all files in LittleFS
    //-----------------------------------------------------------------------

    // Initialize the button pin
    pinMode(BUTTON_PIN, INPUT_PULLDOWN);
    pinMode(tachoInput, INPUT);

    // Initial check in setup to see if BLE server should start right away
    if (checkStartCondition())
    {
        startBLEServer();
        isBleServerStarted = true; // Set flag to indicate BLE server has started
    }
    else
    {
        Serial.println("Condition not met, waiting to start BLE server...");
    }
}

void loop()
{
    rpm = 60000 / duration;
   // Serial.println(duration);

    // Serial.print("rpm= ");
   // Serial.println(rpm);
    //  delay(1000);         //delay already here needed for ble?? test upload??
    //   Serial.print("loop running on core ");
    //   Serial.println(xPortGetCoreID());

    // loop and ble runs running on core 1

    // Only check and start BLE server if it's not already started
    if (!isBleServerStarted)
    {
        if (checkStartCondition())
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
            }
    //---------------------------------------------------------------------------------------------

            if (gps.location.isUpdated())
            {
                unsigned long currentTime = millis();

                if (currentTime - lastGpsLogTime >= gpsLogInterval)
                {
                    lastGpsLogTime = currentTime;
                    String gpsData = "G," + String(currentLat, 6) + "," + String(currentLng, 6) + "," + String(mph, 1) + "," + String(currentTime) + ",";

                    logBuffer += gpsData;

                    // Check if the buffer needs to be flushed
                    if (logBuffer.length() >= bufferSize || millis() - lastFlushTime >= flushInterval)
                    {
                        flushBuffer(logBuffer);
                    }

                    // if (fsHandler.appendData(logId.c_str(), gpsData.c_str()))
                    // {
                    // //    Serial.println("GPS data written to " + logId);
                    // }
                }
            }

            // Temperature Data
            if (millis() - lastTemperatureLogTime >= temperatureLogInterval && gps.location.isUpdated())
            {
                lastTemperatureLogTime = millis();
                float temperature = 65;                                                      // readTemperature();   // later....
                String engTempData = "ET," + String(temperature, 2) + "," + String(millis()) + ","; // Append current time

                logBuffer += engTempData;

                // Check if the buffer needs to be flushed
                if (logBuffer.length() >= bufferSize || millis() - lastFlushTime >= flushInterval)
                {
                    flushBuffer(logBuffer);
                }

                // if (fsHandler.appendData(logId.c_str(), tempData.c_str()))
                // {
                //    // Serial.println("Temperature data written to " + logId);
                // }
            }

            // Rpm Data
             if (millis() - lastRpmLogTime >= rpmLogInterval && gps.location.isUpdated())

            {
                lastRpmLogTime = millis();
                                                                        // readTemperature();   // later....

             String rpmData = "R," + String(rpm) + "," + String(millis()) + ","; // Append current time

                logBuffer += rpmData;

                // Check if the buffer needs to be flushed
                if (logBuffer.length() >= bufferSize || millis() - lastFlushTime >= flushInterval)
                {
                    flushBuffer(logBuffer);
                }
                // if (fsHandler.appendData(logId.c_str(), rpmData.c_str()))
                // {
                //   //  Serial.println("RPM data written to " + logId);
                // }
            }
        }
}