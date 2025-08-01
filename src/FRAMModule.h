#pragma once
#include <Adafruit_FRAM_SPI.h>

// --- Only DECLARATIONS here ---

// FRAM pin config (declared, not defined)
extern uint8_t FRAM_CS1;
extern uint8_t FRAM_CS2;
extern uint8_t FRAM_SCK;
extern uint8_t FRAM_MISO;
extern uint8_t FRAM_MOSI;

// FRAM objects and parameters
extern Adafruit_FRAM_SPI fram1;
extern Adafruit_FRAM_SPI fram2;
extern uint8_t addrSizeInBytes;
extern const size_t HEADER_ADDRESS;
