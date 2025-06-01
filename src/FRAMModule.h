#pragma once
#include <Adafruit_FRAM_SPI.h>

// Pin configuration
extern uint8_t FRAM_CS;
extern uint8_t FRAM_SCK;
extern uint8_t FRAM_MISO;
extern uint8_t FRAM_MOSI;

// FRAM object
extern Adafruit_FRAM_SPI fram;
extern uint8_t addrSizeInBytes;
extern const size_t HEADER_ADDRESS;