#include "ADS1115Module.h"
#include <pgmspace.h>
#include "Settings.h"

// Lookup table for AFR-to-lambda conversion
struct AfrLookup
{
    float voltage;
    float lambda;
};

// const AfrLookup afrTable[] PROGMEM = {
//     {0.5, 0.70},
//     {0.60, 0.80},
//     {1.3, 0.90},
//     {2.0, 1.00},
//     //{2.5, 1.10},
//     {2.5, 1.02},
//     {3.0, 1.15},
//     {4.0, 1.25},
//     {4.5, 1.30},
// };

const AfrLookup afrTable[] PROGMEM = {
    // Z2T linear output, AFR = volts x 2 + 9.6, CONVERT TO LAMBDA / 14.7
    {0.0, 0.65},
    {1.00, 0.78},
    {1.50, 0.85},
    {2.0, 0.92},
    {2.5, 0.99},
    {3.0, 1.06},
    {4.0, 1.19},
    {5.0, 1.33},
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

double readAndCompensateFiltered(ADS1115 &ads, int channel, double r1, double r2)
{

    static float filteredVolts;
    const float alpha = 0.50;

    int16_t raw = ads.readADC(channel);
    double vMeasured = raw * 0.125 / 1000.0;
    vMeasured *((r1 + r2) / r2);
    filteredVolts = alpha * vMeasured + (1.0f - alpha) * filteredVolts;
    return filteredVolts;
}

uint8_t readSensor(ADS1115 &ads, int channel, double r1, double r2)
{
    double voltage = readAndCompensate(ads, channel, r1, r2);
    uint8_t scaled = round(voltage * 10);
    return scaled > 255 ? 255 : scaled;
}

uint8_t readSensorFiltered(ADS1115 &ads, int channel, double r1, double r2)
{
    static float filteredVolts;
    const float alpha = 0.25;
    double voltage = readAndCompensate(ads, channel, r1, r2);
    uint8_t scaled = round(voltage * 10);
    filteredVolts = alpha * scaled + (1.0f - alpha) * filteredVolts;
    return filteredVolts > 255 ? 255 : filteredVolts;
}

double doubleReadSensor(ADS1115 &ads, int channel, double r1, double r2)
{
    double voltage = readAndCompensate(ads, channel, r1, r2);
    // uint8_t scaled = round(voltage * 10);
    return voltage;
}

uint8_t readTps(ADS1115 &ads, int channel)
{

    int16_t raw = ads.readADC(channel);
    if (raw < tpsMin)
        raw = tpsMin;
    if (raw > tpsMax)
        raw = tpsMax;

    // Prevent division by zero
    int delta = tpsMax - tpsMin;
    if (delta <= 0)
    {
        // Serial.println("❌ TPS calibration invalid: tpsMax <= tpsMin");
        return 0;
    }

    uint8_t val = (uint32_t)(raw - tpsMin) * 100 / (tpsMax - tpsMin);
    return val > 100 ? 100 : val;
}

double scaleVoltage(int16_t raw, double r1, double r2)
{
    double vMeasured = raw * 0.125 / 1000.0;
    return vMeasured * ((r1 + r2) / r2);
}

uint8_t mapLambda(ADS1115 &ads, int channel, double r1, double r2)
{
    static float filteredLambda; //  0.88 = 12.93, Start at λ = 1.0 (stoich) = 14.7:1
    const float alpha = 0.50;    // Tune for desired smoothing

    float v = readAndCompensate(ads, channel, r1, r2);

    AfrLookup entry1,
        entry2;
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
    // return round(lambda * 100); // Store lambda
    // return lambda; // Store lambda
}

uint8_t zeitronixLambda(ADS1115 &ads, int channel, double r1, double r2)
{

    static float filteredLambda; //  0.88 = 12.93, Start at λ = 1.0 (stoich) = 14.7:1
    const float alpha = 0.75;    // Tune for desired smoothing

    float v = readAndCompensate(ads, channel, r1, r2);

    float zeetronixTransferFunction = (v * 2 + 9.6) / 14.7;

    // Apply low-pass filter
    filteredLambda = alpha * zeetronixTransferFunction + (1.0f - alpha) * filteredLambda;

    return round(filteredLambda * 100); // Store lambda * 100 for compact logging
}
