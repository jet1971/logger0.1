#ifndef FRAM_LOG_HEADER_H
#define FRAM_LOG_HEADER_H

#include <stdint.h>

struct FramLogHeader
{
    char fileName[32];
    uint32_t dataLength;
};

// Extern declaration so it can be shared
//(just a promise, not a definition)
extern FramLogHeader header;
extern FramLogHeader header2;

#endif // FRAM_LOG_HEADER_H
