#include "TemperatureFunction.h"
#include <Preferences.h>

Preferences airTempSensorCalibration; // Create a Preferences instance

const int NUM_POINTS = 19;
float airTemperatureVoltagePoints[NUM_POINTS] = {0}; // iniatailise with zeros instead of garbage
float airTemperaturePoints[NUM_POINTS] = {0};

double scaleAirVolts(int16_t raw)
{
    double vMeasured = raw * 0.125 / 1000.0;
    return vMeasured;
}

int interpolateAirTemperature(ADS1115 &ads, int channel)
{

    int16_t raw = ads.readADC(channel);
    float voltage = scaleAirVolts(raw);


    if (voltage >= airTemperatureVoltagePoints[0]) // CLAMPS TEMP
        return airTemperaturePoints[0];
    if (voltage <= airTemperatureVoltagePoints[NUM_POINTS - 1])
        return airTemperaturePoints[NUM_POINTS - 1]; // CLAMPS TEMP

    for (int i = 0; i < NUM_POINTS - 1; i++)
    {
        if (voltage <= airTemperatureVoltagePoints[i] && voltage > airTemperatureVoltagePoints[i + 1])
        {
            float slope = (airTemperaturePoints[i + 1] - airTemperaturePoints[i]) / (airTemperatureVoltagePoints[i + 1] - airTemperatureVoltagePoints[i]);
            float temp = airTemperaturePoints[i] + slope * (voltage - airTemperatureVoltagePoints[i]);
            return round(temp); // Optional: round to nearest int
        }
    }

    return -999; // Should not happen if voltage is within range
}

void loadAirTemperatureCalibration()
{
    airTempSensorCalibration.begin("airTempCalibration", true);                           // Open preferences for logger settings, true = read only
    airTemperatureVoltagePoints[0] = airTempSensorCalibration.getFloat("voltagePoint0", 2.793); // -10 degrees        Then set from the APP
    airTemperatureVoltagePoints[1] = airTempSensorCalibration.getFloat("voltagePoint1", 2.668); // Default ??
    airTemperatureVoltagePoints[2] = airTempSensorCalibration.getFloat("voltagePoint2", 2.525); // Default ??
    airTemperatureVoltagePoints[3] = airTempSensorCalibration.getFloat("voltagePoint3", 2.366); // Default ?
    airTemperatureVoltagePoints[4] = airTempSensorCalibration.getFloat("voltagePoint4", 2.195); // Default ??
    airTemperatureVoltagePoints[5] = airTempSensorCalibration.getFloat("voltagePoint5", 2.016); // Default ?
    airTemperatureVoltagePoints[6] = airTempSensorCalibration.getFloat("voltagePoint6", 1.832); // Default ?
    airTemperatureVoltagePoints[7] = airTempSensorCalibration.getFloat("voltagePoint7", 1.650); // Default ??        Then set from the APP
    airTemperatureVoltagePoints[8] = airTempSensorCalibration.getFloat("voltagePoint8", 1.473); // Default ??
    airTemperatureVoltagePoints[9] = airTempSensorCalibration.getFloat("voltagePoint9", 1.304); // Default ??
    airTemperatureVoltagePoints[10] = airTempSensorCalibration.getFloat("voltagePoint10", 1.147); // Default ?
    airTemperatureVoltagePoints[11] = airTempSensorCalibration.getFloat("voltagePoint11", 1.004); // Default ??
    airTemperatureVoltagePoints[12] = airTempSensorCalibration.getFloat("voltagePoint12", 0.875); // Default ?
    airTemperatureVoltagePoints[13] = airTempSensorCalibration.getFloat("voltagePoint13", 0.759);
    airTemperatureVoltagePoints[14] = airTempSensorCalibration.getFloat("voltagePoint14", 0.658); // Default ?
    airTemperatureVoltagePoints[15] = airTempSensorCalibration.getFloat("voltagePoint15", 0.569); // Default ??
    airTemperatureVoltagePoints[16] = airTempSensorCalibration.getFloat("voltagePoint16", 0.492); // Default ?
    airTemperatureVoltagePoints[17] = airTempSensorCalibration.getFloat("voltagePoint17", 0.426);
    airTemperatureVoltagePoints[18] = airTempSensorCalibration.getFloat("voltagePoint18", 0.368); // 80 degrees

    airTemperaturePoints[0] = airTempSensorCalibration.getFloat("tempPoint0", -10); // NOT REALLY NEEDED, THESE COULD BE JUST INITIASLISED IN A STATIC ARRAY?
    airTemperaturePoints[1] = airTempSensorCalibration.getFloat("tempPoint1", -5); // Default ??
    airTemperaturePoints[2] = airTempSensorCalibration.getFloat("tempPoint2", 0); // Default ??
    airTemperaturePoints[3] = airTempSensorCalibration.getFloat("tempPoint3", 5); // Default ?
    airTemperaturePoints[4] = airTempSensorCalibration.getFloat("tempPoint4", 10); // Default ??
    airTemperaturePoints[5] = airTempSensorCalibration.getFloat("tempPoint5", 15); // Default ?
    airTemperaturePoints[6] = airTempSensorCalibration.getFloat("tempPoint6", 20); // Default ?
    airTemperaturePoints[7] = airTempSensorCalibration.getFloat("tempPoint7", 25); // Default ?
    airTemperaturePoints[8] = airTempSensorCalibration.getFloat("tempPoint8", 30); // Default ??
    airTemperaturePoints[9] = airTempSensorCalibration.getFloat("tempPoint9", 35); // Default ?
    airTemperaturePoints[10] = airTempSensorCalibration.getFloat("tempPoint10", 40); // Default ?
    airTemperaturePoints[11] = airTempSensorCalibration.getFloat("tempPoint11", 45);  // Default ?
    airTemperaturePoints[12] = airTempSensorCalibration.getFloat("tempPoint12", 50);  // Default ?
    airTemperaturePoints[13] = airTempSensorCalibration.getFloat("tempPoint13", 55);  // Default ?
    airTemperaturePoints[14] = airTempSensorCalibration.getFloat("tempPoint14", 60);  // Default ??
    airTemperaturePoints[15] = airTempSensorCalibration.getFloat("tempPoint15", 65);  // Default ?
    airTemperaturePoints[16] = airTempSensorCalibration.getFloat("tempPoint16", 70); // Default ?
    airTemperaturePoints[17] = airTempSensorCalibration.getFloat("tempPoint17", 75); // Default ?
    airTemperaturePoints[18] = airTempSensorCalibration.getFloat("tempPoint18", 80); // Default ?
    airTempSensorCalibration.end();
}

void saveAirTemperatureCalibration()
{
    airTempSensorCalibration.begin("airTempCalibration", false); // write mode
    airTempSensorCalibration.putFloat("voltagePoint0", airTemperatureVoltagePoints[0]);
    airTempSensorCalibration.putFloat("voltagePoint1", airTemperatureVoltagePoints[1]);
    airTempSensorCalibration.putFloat("voltagePoint2", airTemperatureVoltagePoints[2]);
    airTempSensorCalibration.putFloat("voltagePoint3", airTemperatureVoltagePoints[3]);
    airTempSensorCalibration.putFloat("voltagePoint4", airTemperatureVoltagePoints[4]);
    airTempSensorCalibration.putFloat("voltagePoint5", airTemperatureVoltagePoints[5]);
    airTempSensorCalibration.putFloat("voltagePoint6", airTemperatureVoltagePoints[6]);
    airTempSensorCalibration.putFloat("voltagePoint7", airTemperatureVoltagePoints[7]);
    airTempSensorCalibration.putFloat("voltagePoint8", airTemperatureVoltagePoints[8]);
    airTempSensorCalibration.putFloat("voltagePoint9", airTemperatureVoltagePoints[9]);
    airTempSensorCalibration.putFloat("voltagePoint10", airTemperatureVoltagePoints[10]);
    airTempSensorCalibration.putFloat("voltagePoint11", airTemperatureVoltagePoints[11]);
    airTempSensorCalibration.putFloat("voltagePoint12", airTemperatureVoltagePoints[12]);
    airTempSensorCalibration.putFloat("voltagePoint13", airTemperatureVoltagePoints[13]);
    airTempSensorCalibration.putFloat("voltagePoint14", airTemperatureVoltagePoints[14]);
    airTempSensorCalibration.putFloat("voltagePoint15", airTemperatureVoltagePoints[15]);
    airTempSensorCalibration.putFloat("voltagePoint16", airTemperatureVoltagePoints[16]);
    airTempSensorCalibration.putFloat("voltagePoint17", airTemperatureVoltagePoints[17]);
    airTempSensorCalibration.putFloat("voltagePoint18", airTemperatureVoltagePoints[18]);   // 80 degrees

    airTempSensorCalibration.putFloat("tempPoint0", -10); // NOT REALLY NEEDED, THESE COULD BE JUST INITIASLISED IN A STATIC ARRAY?
    airTempSensorCalibration.putFloat("tempPoint1", -5); // Default ??
    airTempSensorCalibration.putFloat("tempPoint2", 0); // Default ??
    airTempSensorCalibration.putFloat("tempPoint3", 5); // Default ?
    airTempSensorCalibration.putFloat("tempPoint4", 10); // Default ??
    airTempSensorCalibration.putFloat("tempPoint5", 15); // Default ?    
    airTempSensorCalibration.putFloat("tempPoint6", 20); // Default ?
    airTempSensorCalibration.putFloat("tempPoint7", 25); // Default ?
    airTempSensorCalibration.putFloat("tempPoint8", 30); // Default ??
    airTempSensorCalibration.putFloat("tempPoint9", 35); // Default ?
    airTempSensorCalibration.putFloat("tempPoint10", 40); // Default ?
    airTempSensorCalibration.putFloat("tempPoint11", 45);  // Default ?
    airTempSensorCalibration.putFloat("tempPoint12", 50);  // Default ?
    airTempSensorCalibration.putFloat("tempPoint13", 55);  // Default ?
    airTempSensorCalibration.putFloat("tempPoint14", 60);  // Default ??        
    airTempSensorCalibration.putFloat("tempPoint15", 65);  // Default ?
    airTempSensorCalibration.putFloat("tempPoint16", 70); // Default ?
    airTempSensorCalibration.putFloat("tempPoint17", 75); // Default ?  
    airTempSensorCalibration.putFloat("tempPoint18", 80); // Default ?
    airTempSensorCalibration.end();
}