#ifndef LITTLEFSHANDLER_H
#define LITTLEFSHANDLER_H

#include <Arduino.h>
#include <LittleFS.h> // Include LittleFS

class LittleFSHandler
{
public:
    LittleFSHandler();
    bool begin();

    void formatLittleFS();

    bool writeData(const char *filename, const uint8_t *data, size_t length);

    bool appendData(const char *filename, const char *data);

    String readData(const char *filename);
    
    //fs::File openFile(const char *filename, const char *mode);
    fs::File openFile(const char *filename);
    size_t getFileSize(const char *filename);
    String listFiles();
};

#endif // LITTLEFSHANDLER_H
