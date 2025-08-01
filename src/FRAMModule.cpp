#include "FRAMModule.h"

// --- DEFINITIONS with initial values ---

uint8_t FRAM_CS1 = 33;
uint8_t FRAM_CS2 = 32;
uint8_t FRAM_SCK = 18;
uint8_t FRAM_MISO = 19;
uint8_t FRAM_MOSI = 23;

Adafruit_FRAM_SPI fram1 = Adafruit_FRAM_SPI(FRAM_SCK, FRAM_MISO, FRAM_MOSI, FRAM_CS1);
Adafruit_FRAM_SPI fram2 = Adafruit_FRAM_SPI(FRAM_SCK, FRAM_MISO, FRAM_MOSI, FRAM_CS2);

uint8_t addrSizeInBytes = 3;
const size_t HEADER_ADDRESS = 0x00;