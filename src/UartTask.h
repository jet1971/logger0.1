#ifndef UARTTASK_H
#define UARTTASK_H

#include "Arduino.h"

extern char uartBuffer[16]; // "extern" = tells the compiler it exists somewhere
extern uint8_t uartIndex;
extern uint32_t rpm; // rpm variable will be defined in .cpp

void uartTask(); // Function prototype

#endif // UARTTASK_H
