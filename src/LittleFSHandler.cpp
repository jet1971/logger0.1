#include "LittleFSHandler.h"
#include "FS.h"
#include <ArduinoJson.h>

// Constructor
LittleFSHandler::LittleFSHandler()
{
    // Constructor body can be empty for now
}

// Initialize LittleFS
bool LittleFSHandler::begin()
{
    return LittleFS.begin();
}
//------------------------------------------------------------------------------------------------

// Write data to a file
bool LittleFSHandler::writeData(const char *filename, const char *data)
{
    File file = LittleFS.open(filename, FILE_WRITE, true);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return false;
    }

    file.print(data); // Write data to the file
    file.close();
    Serial.println("Data written successfully");
    return true;
}

bool LittleFSHandler::appendData(const char *filename, const char *data)
{
    File file = LittleFS.open(filename, FILE_APPEND); // Open in append mode
    if (!file)
    {
        Serial.println("Failed to open file for appending");
        return false;
    }
    file.print(data); // Append the data
    file.close();
    return true;
}

//------------------------------------------------------------------------------------------------
// Read data from a file
String LittleFSHandler::readData(const char *filename)
{
    File file = LittleFS.open(filename, FILE_READ, true);
    if (!file)
    {
        Serial.println("Failed to open file for reading");
        return "";
    }

    String data = file.readString(); // Read the file contents
    file.close();
    return data;
}

//------------------------------------------------------------------------------------------------
fs::File LittleFSHandler::openFile(const char *filename)
{
    fs::File file = LittleFS.open(filename, FILE_READ, true);
    // size_t fileSize = file.size();
    // Serial.print("This is the file size : ");
    // Serial.println(fileSize); // don't need here anymore

    if (!file)
    {
        Serial.println("Failed to open file");
    }
    return file;
}

//------------------------------------------------------------------------------------------------
size_t LittleFSHandler::getFileSize(const char *filename)
{
    File file = LittleFS.open(filename, FILE_READ, true);
    if (!file)
    {
        Serial.println("Failed to open file to get size");
        return 0; // Return 0 if the file could not be opened
    }

    size_t fileSize = file.size(); // Get the file size
    file.close();                  // Close the file after getting the size
    return fileSize;
}

//------------------------------------------------------------------------------------------------

String LittleFSHandler::listFiles()
{

    JsonDocument doc;
    JsonArray filesArray = doc["files"].to<JsonArray>();

    // JsonArray filesArray = doc.createNestedArray("files"); // Create a JSON array to store all file details
    //   JsonObject obj = doc.to<JsonObject>();
    // JsonObject fileObj = doc.to<JsonObject>();

    File root = LittleFS.open("/"); // Open the root directory
    if (!root || !root.isDirectory())
    {
        Serial.println("Failed to open root directory");
        return ""; // Return an empty string if there's an error
    }

    File file = root.openNextFile(); // Open the first file

    while (file)
    {
        String temp = file.name();
        size_t fileSize = file.size();

        // Find positions based on expected format
        int colonPos = temp.indexOf(':');  // Locate the time part by finding the colon
        int txtPos = temp.indexOf(".txt"); // Locate the end of the file extension

        // Extract the time part based on the colon
        String time = temp.substring(colonPos - 2, colonPos + 3); // Extract "HH:MM"
                                                                  // na141020248:51.txt
        // Extract the venue, year, month, and day, String to date eg: cp1104202413:44.txt
        String venue = temp.substring(colonPos - 12, colonPos - 10); // Extract venue initials, eg (cp = Cadwell Park), (na = not available)
        String year = temp.substring(colonPos - 6, colonPos - 2);    // Extract year (2024)
        String month = temp.substring(colonPos - 8, colonPos - 6);   // Extract month (10)
        String day = temp.substring(colonPos - 10, colonPos - 8);    // Extract day
        String fastestLap = "00:00.00";                              // temp value

        // Create a temporary JSON object to store each file's details
        // JsonObject fileObj = filesArray.createNestedObject();
        JsonObject fileObj = filesArray.add<JsonObject>();
        String venueName;

        if (venue == "ho")
        {
            venueName = "Home";
        }
        else if (venue == "wo")
        {
            venueName = "Work";
        }
        else
        {
            venueName = "Venue N/A";
        }

        fileObj["venue"] = venueName;
        fileObj["day"] = day;
        fileObj["month"] = month;
        fileObj["year"] = year;
        fileObj["time"] = time;
        fileObj["fileSize"] = fileSize;
        fileObj["fastestLap"] = fastestLap;
        fileObj["fileName"] = temp;


        Serial.print("File: ");
        Serial.println(file.name());
        // Serial.print(" ");
        // Serial.print("Size: ");
        // Serial.println(file.size());

        file = root.openNextFile(); // Get the next file
    };

    // Serialize the entire JSON object
    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString; // Return the JSON string with all file details
}

//------------------------------------------------------------------------------------------------