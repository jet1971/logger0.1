#include "Tacho.h"
#include "Settings.h"

// ==== CONFIGURATION ====
int TACHO_PIN = 13;                         // Pin connected to Tacho signal
const int maxSamples = 5;                   // For moving average
const unsigned long pulseTimeout = 1000000; // 1 second = 1,000,000 µs


// ==== INTERNAL STATE ====
unsigned long lastPulseTime = 0;
unsigned long currentPulseTime = 0;
unsigned long pulseInterval = 0;
uint32_t rpm = 0;
bool lastState = LOW;

unsigned long intervals[maxSamples] = {0};
int sampleIndex = 0;
bool filled = false;


void tacho()
{
    
    {
        bool currentState = digitalRead(TACHO_PIN);

        // Rising edge detection
        if (currentState == HIGH && lastState == LOW)
        {
            currentPulseTime = micros();

            if (lastPulseTime > 0)
            {
                pulseInterval = currentPulseTime - lastPulseTime;

                // Reject invalid pulses
                if (pulseInterval > 1050 && pulseInterval < 200000) // 3750 for x2 multiplier, (Harrison zx7R) 7500 µs = 16000 RPM, 200000 µs = 600 RPM if multiplier is 2
                {
                    intervals[sampleIndex] = pulseInterval;
                    sampleIndex = (sampleIndex + 1) % maxSamples;
                    if (sampleIndex == 0)
                        filled = true;

                    // Moving average
                    unsigned long sum = 0;
                    int count = filled ? maxSamples : sampleIndex;
                    for (int i = 0; i < count; i++)
                    {
                        sum += intervals[i];
                    }

                    unsigned long avgInterval = sum / count;
                    rpm = (30UL * 1000000UL) / avgInterval * rpmMultiplier;
                }
            }

            lastPulseTime = currentPulseTime;
        }

        lastState = currentState;

        // Timeout check: no pulse = zero RPM
        if ((micros() - lastPulseTime) > pulseTimeout && lastPulseTime != 0)
        {
            rpm = 0;
            lastPulseTime = 0;
        }


           // Serial.println(rpm); // debug

        }
    }
 
