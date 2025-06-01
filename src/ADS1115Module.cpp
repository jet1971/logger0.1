#include "ADS1115Module.h"
#include <pgmspace.h>

// Lookup table for AFR-to-lambda conversion
struct AfrLookup
{
    float voltage;
    float lambda;
};

const AfrLookup afrTable[] PROGMEM = {
    {0.5, 0.70},
    {0.60, 0.80},
    {1.3, 0.90},
    {2.0, 1.00},
    {2.5, 1.10},
    {3.0, 1.15},
    {4.0, 1.25},
    {4.5, 1.30},
};

const uint8_t afrTableSize = sizeof(afrTable) / sizeof(afrTable[0]);

ADS1115 ADS1(0x48);
ADS1115 ADS2(0x49);

void setupADS()
{
    ADS1.begin();
    ADS1.setGain(1);
    ADS1.setDataRate(7);
    ADS1.setWireClock(400000);

    ADS2.begin();
    ADS2.setGain(1);
    ADS2.setDataRate(7);
    ADS2.setWireClock(400000);
}

double readAndCompensate(ADS1115 &ads, int channel, double r1, double r2)
{
    int16_t raw = ads.readADC(channel);
    double vMeasured = raw * 0.125 / 1000.0;
    return vMeasured * ((r1 + r2) / r2);
}

uint8_t readSensor(ADS1115 &ads, int channel, double r1, double r2)
{
    double voltage = readAndCompensate(ads, channel, r1, r2);
    uint8_t scaled = round(voltage * 10);
    return scaled > 255 ? 255 : scaled;
}

uint8_t readTps(ADS1115 &ads, int channel)
{
    const int tpsMin = 3458; // 0.6V ISH CALIBRATION VALUES, BIKE SPECIFIC, DO IN APP EVENTUALLY
    const int tpsMax = 20000; // 4.6V ISH

    int16_t raw = ads.readADC(channel);
    if (raw < tpsMin)
        raw = tpsMin;
    if (raw > tpsMax)
        raw = tpsMax;

    uint8_t val = (uint32_t)(raw - tpsMin) * 100 / (tpsMax - tpsMin);
    return val > 100 ? 100 : val;
}

double scaleVoltage(int16_t raw, double r1, double r2)
{
    double vMeasured = raw * 0.125 / 1000.0;
    return vMeasured * ((r1 + r2) / r2);
}

// uint8_t mapLambda(ADS1115 &ads, int channel, double r1, double r2)
// {
//     int16_t raw = ads.readADC(channel);
//     float v = scaleVoltage(raw, r1, r2);

//     AfrLookup entry1, entry2;
//     if (v <= pgm_read_float(&afrTable[0].voltage))
//     {
//         float lambda = pgm_read_float(&afrTable[0].lambda);
//         return round(lambda * 100);
//     }

//     for (uint8_t i = 0; i < afrTableSize - 1; ++i)
//     {
//         entry1.voltage = pgm_read_float(&afrTable[i].voltage);
//         entry1.lambda = pgm_read_float(&afrTable[i].lambda);
//         entry2.voltage = pgm_read_float(&afrTable[i + 1].voltage);
//         entry2.lambda = pgm_read_float(&afrTable[i + 1].lambda);

//         if (v >= entry1.voltage && v <= entry2.voltage)
//         {
//             float ratio = (v - entry1.voltage) / (entry2.voltage - entry1.voltage);
//             float lambda = entry1.lambda + ratio * (entry2.lambda - entry1.lambda);
//             return round(lambda * 100);
//         }
//     }

//     float lambdaMax = pgm_read_float(&afrTable[afrTableSize - 1].lambda);
//     return round(lambdaMax * 100);
// }

uint8_t mapLambda(ADS1115 &ads, int channel, double r1, double r2)
{
    static float filteredLambda = 1.0; // Start at λ = 1.0 (stoich)
    const float alpha = 0.01;           // Tune for desired smoothing

    int16_t raw = ads.readADC(channel);
    float v = scaleVoltage(raw, r1, r2);

    AfrLookup entry1, entry2;
    float lambda = 1.0;

    if (v <= pgm_read_float(&afrTable[0].voltage))
    {
        lambda = pgm_read_float(&afrTable[0].lambda);
    }
    else
    {
        for (uint8_t i = 0; i < afrTableSize - 1; ++i)
        {
            entry1.voltage = pgm_read_float(&afrTable[i].voltage);
            entry1.lambda = pgm_read_float(&afrTable[i].lambda);
            entry2.voltage = pgm_read_float(&afrTable[i + 1].voltage);
            entry2.lambda = pgm_read_float(&afrTable[i + 1].lambda);

            if (v >= entry1.voltage && v <= entry2.voltage)
            {
                float ratio = (v - entry1.voltage) / (entry2.voltage - entry1.voltage);
                lambda = entry1.lambda + ratio * (entry2.lambda - entry1.lambda);
                break;
            }
        }

        // Clamp to upper limit
        if (v > pgm_read_float(&afrTable[afrTableSize - 1].voltage))
        {
            lambda = pgm_read_float(&afrTable[afrTableSize - 1].lambda);
        }
    }

    // Apply low-pass filter
    filteredLambda = alpha * lambda + (1.0f - alpha) * filteredLambda;

    return round(filteredLambda * 100); // Store lambda * 100 for compact logging
}
