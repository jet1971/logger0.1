 #ifndef ADS1115MODULE_H
 #define ADS1115MODULE_H

//#include <Arduino.h>
#include "ADS1X15.h"


extern ADS1115 ADS1; // Declare the ADS object
extern ADS1115 ADS2; // Declare the ADS object

void setupADS();

inline double readAndCompensate(ADS1115 &ads, int channel, double r1, double r2);
uint8_t readSensor(ADS1115 &ads, int channel, double r1, double r2);
uint8_t readTps(ADS1115 &ads, int channel);


#endif // ADS1115MODULE_H

