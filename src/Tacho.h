#ifndef TACHO_H
#define TACHO_H

#include "Arduino.h"

extern int TACHO_PIN;
extern uint32_t rpm; // rpm variable will be defined in .cpp

void tacho(); // Function prototype

#endif 