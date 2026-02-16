#ifndef HEADER_H
#define HEADER_H

#include <stdint.h>

typedef struct {   // header declaration
    uint64_t time_stamp_ms;
    uint8_t sensor_id;
    uint8_t node_id;
    uint8_t len;
} Header;

#endif