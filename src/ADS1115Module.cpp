#include "ADS1115Module.h"

ADS1115 ADS1(0x48); // Initialize ADS1115 with I2C address 0x48: ADDR -> GND
ADS1115 ADS2(0x49); // 0x49: ADDR -> VDD

void setupADS()
{
    ADS1.begin();
    ADS1.setGain(1);           // 1: 4.096V max voltage
    ADS1.setDataRate(7);       // 860SPS
    ADS1.setWireClock(400000); // 400kHz

    ADS2.begin();
    ADS2.setGain(1);           // 1: 4.096V max voltage
    ADS2.setDataRate(7);       // 860SPS
    ADS2.setWireClock(400000); // 400kHz
}

// General function to read and compensate a voltage input
inline double readAndCompensate(ADS1115 &ads, int channel, double r1, double r2)
{
    int16_t raw = ads.readADC(channel);
    double vMeasured = raw * 0.125 / 1000.0; // 0.125mV per bit
    return vMeasured * ((r1 + r2) / r2);
}

// Function for a sensor scaled down with eg, 3.3k / 6.8k
uint8_t readSensor(ADS1115 &ads, int channel, double r1, double r2)
{

   double voltage = readAndCompensate(ads, channel, r1, r2);         //eg, 3.3k / 6.8k
   uint8_t scaled = round(voltage * 10);                             // e.g. 4.85V → 49
   return scaled > 255 ? 255 : scaled;                               // Clamp to 0–255
}

// Define these (later replace these with values saved from app)
const int tpsMin = 0;  // Minimum raw ADC value (0.5V ≈ 500mV)
// const int tpsMax = 6612; // Maximum raw ADC value (4.5V ≈ 4500mV)????
const int tpsMax = 4200; // Maximum raw ADC value (4.5V ≈ 4500mV)????

uint8_t readTps(ADS1115 &ads, int channel)
{
    int16_t raw = ads.readADC(channel);

    // Clamp between the calibrated min and max
    if (raw < tpsMin)
        raw = tpsMin;
    if (raw > tpsMax)
        raw = tpsMax;

    // Map raw range [tpsMin–tpsMax] to [0–100] %
    uint8_t val = (uint32_t)(raw - tpsMin) * 100 / (tpsMax - tpsMin);

    if (val > 100)
        val = 100; // Safety clamp

    return val;
}

//-----------------------------------------------------------------------------------------------------
