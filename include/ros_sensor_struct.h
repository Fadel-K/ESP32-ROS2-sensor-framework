#include "header.h"

struct GeneralSensor {
    Header header;
    uint8_t data[];
};