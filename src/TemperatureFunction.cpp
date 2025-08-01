#include "TemperatureFunction.h"
#include <Preferences.h>

Preferences engineTempSensorCalibration; // Create a Preferences instance

const int NUM_POINTS = 7; 
float voltagePoints[NUM_POINTS] = {0}; // iniatailise with zeros instead of garbage
float tempPoints[NUM_POINTS] = {0};

double scaleVolts(int16_t raw, double r1, double r2) // Adjust output for given resistor combination
{
    double vMeasured = raw * 0.125 / 1000.0;
    return vMeasured * ((r1 + r2) / r2);
}

int interpolateEngineTemperature(ADS1115 &ads, int channel, double r1, double r2)
{

    int16_t raw = ads.readADC(channel);
    float voltage = scaleVolts(raw, r1, r2);

    if (voltage >= voltagePoints[0]) // CLAMPS TEMP
        return tempPoints[0];
    if (voltage <= voltagePoints[NUM_POINTS - 1])
        return tempPoints[NUM_POINTS - 1]; // CLAMPS TEMP

    for (int i = 0; i < NUM_POINTS - 1; i++)
    {
        if (voltage <= voltagePoints[i] && voltage > voltagePoints[i + 1])
        {
            float slope = (tempPoints[i + 1] - tempPoints[i]) / (voltagePoints[i + 1] - voltagePoints[i]);
            float temp = tempPoints[i] + slope * (voltage - voltagePoints[i]);
            return round(temp); // Optional: round to nearest int
        }
    }

    return -999; // Should not happen if voltage is within range
}

void loadEngineTemperatureCalibration()
{
    engineTempSensorCalibration.begin("eTCalibration", true); // Open preferences for logger settings, true = read only
    voltagePoints[0] = engineTempSensorCalibration.getFloat("voltagePoint0", 4.500); // Default ??        Then set from the APP
    voltagePoints[1] = engineTempSensorCalibration.getFloat("voltagePoint1", 4.050); // Default ??
    voltagePoints[2] = engineTempSensorCalibration.getFloat("voltagePoint2", 3.328); // Default ??
    voltagePoints[3] = engineTempSensorCalibration.getFloat("voltagePoint3", 2.570); // Default ?
    voltagePoints[4] = engineTempSensorCalibration.getFloat("voltagePoint4", 1.842); // Default ??
    voltagePoints[5] = engineTempSensorCalibration.getFloat("voltagePoint5", 1.268); // Default ?
    voltagePoints[6] = engineTempSensorCalibration.getFloat("voltagePoint6", 0.865); // Default ?

    tempPoints[0] = engineTempSensorCalibration.getFloat("tempPoint0", 0); // NOT REALLY NEEDED, THESE COULD BE JUST INITIASLISED IN A STATIC ARRAY?
    tempPoints[1] = engineTempSensorCalibration.getFloat("tempPoint1", 20); // Default ??
    tempPoints[2] = engineTempSensorCalibration.getFloat("tempPoint2", 40); // Default ??
    tempPoints[3] = engineTempSensorCalibration.getFloat("tempPoint3", 60); // Default ?
    tempPoints[4] = engineTempSensorCalibration.getFloat("tempPoint4", 80); // Default ??
    tempPoints[5] = engineTempSensorCalibration.getFloat("tempPoint5", 100); // Default ?
    tempPoints[6] = engineTempSensorCalibration.getFloat("tempPoint6", 120); // Default ?
    engineTempSensorCalibration.end();
}

void saveEngineTemperatureCalibration()
{
    engineTempSensorCalibration.begin("eTCalibration", false); // write mode
    engineTempSensorCalibration.putFloat("voltagePoint0", voltagePoints[0]);
    engineTempSensorCalibration.putFloat("voltagePoint1", voltagePoints[1]);
    engineTempSensorCalibration.putFloat("voltagePoint2", voltagePoints[2]);
    engineTempSensorCalibration.putFloat("voltagePoint3", voltagePoints[3]);
    engineTempSensorCalibration.putFloat("voltagePoint4", voltagePoints[4]);
    engineTempSensorCalibration.putFloat("voltagePoint5", voltagePoints[5]);
    engineTempSensorCalibration.putFloat("voltagePoint6", voltagePoints[6]);

    engineTempSensorCalibration.putFloat("tempPoint0", 0);
    engineTempSensorCalibration.putFloat("tempPoint1", 20);
    engineTempSensorCalibration.putFloat("tempPoint2", 40);
    engineTempSensorCalibration.putFloat("tempPoint3", 60);
    engineTempSensorCalibration.putFloat("tempPoint4", 80);
    engineTempSensorCalibration.putFloat("tempPoint5", 100);
    engineTempSensorCalibration.putFloat("tempPoint6", 120);
    engineTempSensorCalibration.end();
}