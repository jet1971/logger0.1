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

bool LittleFSHandler::writeData(const char *filename, const uint8_t *data, size_t length)
{
    File file = LittleFS.open(filename, FILE_APPEND);   
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return false;
    }

    size_t written = file.write(data, length); // Write exactly 'length' bytes
    //file.close();

    if (written == length)
    {
      //  Serial.println("Data written successfully");
        return true;
    }
    else
    {
        Serial.println("Failed to write all data");
        return false;
    }
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
       // int colonPos = temp.indexOf(':');  // Locate the time part by finding the colon
        int txtPos = temp.indexOf(".txt"); // Locate the end of the file extension

        // Extract the time part based on the colon
        String time = temp.substring(txtPos - 12, txtPos - 8); // Extract "HH:MM"
                                                              // na141020248:51.txt
        // Extract the venue, year, month, and day, String to date eg: cp1104202413:44.txt
        String venue = temp.substring(txtPos - 26, txtPos - 25); // Extract venue initials, eg (cp = Cadwell Park), (na = not available)
        String year = temp.substring(txtPos - 4, txtPos);    // Extract year (2024)
        String month = temp.substring(txtPos - 6, txtPos - 4);       // Extract month (10)
        String day = temp.substring(txtPos - 8 , txtPos - 6);        // Extract day
        String fastestLap = temp.substring(txtPos - 19, txtPos - 12);
        String fastestLapNumber = temp.substring(txtPos - 21, txtPos - 19);
        String id = temp.substring(txtPos - 25, txtPos - 22);
        String version = temp.substring(txtPos - 22, txtPos - 21);

        // Create a temporary JSON object to store each file's details
        // JsonObject fileObj = filesArray.createNestedObject();
        JsonObject fileObj = filesArray.add<JsonObject>();
        String venueName;

        if (venue == "1")
        {
            venueName = "Work";
        }
        else if (venue == "2")
        {
            venueName = "Home";
        }
        else if (venue == "3")
        {
            venueName = "Aintree";
        }
        else if (venue == "4")
        {
            venueName = "Services";
        }
        else if (venue == "5")
        {
            venueName = "Cadwell Park";
        }
        else if (venue == "6")
        {
            venueName = "Oulton Park";
        }
        else if (venue == "7")
        {
            venueName = "IOM";
        }
        else if (venue == "8")
        {
            venueName = "Brands Hatch";
        }
        else if (venue == "9")
        {
            venueName = "Snetterton";
        }
        else if (venue == "a")
        {
            venueName = "Three Sisters";
        }
        else if (venue == "0")
        {
            venueName = "Unknown Venue";
        }
        else
        {
            venueName = "Unknown Venue";
        }

        fileObj["venue"] = venueName;
        fileObj["day"] = day;
        fileObj["month"] = month;
        fileObj["year"] = year;
        fileObj["time"] = time;
        fileObj["fileSize"] = fileSize;
        fileObj["fastestLap"] = fastestLap;
        fileObj["fileName"] = temp;
        fileObj["fastestLapNumber"] = fastestLapNumber;
        fileObj["id"] = id;
        fileObj["version"] = version;



        Serial.print("File: ");
        Serial.println(file.name());
        Serial.print(" ");
        Serial.print("Size: ");
        Serial.println(file.size());

        file = root.openNextFile(); // Get the next file
    };

    // Serialize the entire JSON object
    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString; // Return the JSON string with all file details
}

// //----------------------------------------------------------------------------------------------
void LittleFSHandler::formatLittleFS()
{
    Serial.println("Formatting LittleFS...");
    if (LittleFS.format())
    {
        Serial.println("LittleFS formatted successfully.");
    }
    else
    {
        Serial.println("Failed to format LittleFS.");
    }
}

// //----------------------------------------------------------------------------------------------