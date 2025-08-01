// #include "UartTask.h"
// #include "Settings.h"

// char uartBuffer[16] = {0};
// uint8_t uartIndex = 0;
// uint32_t rpm = 0;

// void uartTask()
// {
//     if (Serial1.available())
//     {
//         char incoming = Serial1.read();

//         if (incoming == '\n')
//         {
//             uartBuffer[uartIndex] = '\0'; // Null terminate
//             int value = atoi(uartBuffer);
//             // if (value > 200 && value < 20000)
//             if (value < 16000)
//             {
//                 rpm = value * rpmMultiplier;
//             }
//             uartIndex = 0; // Reset for next message
//         }
//         else if (uartIndex < sizeof(uartBuffer) - 1)
//         {
//             uartBuffer[uartIndex++] = incoming;
//         }
//         else
//         {
//             uartIndex = 0; // Overflow, reset
//         }
//     }
// }