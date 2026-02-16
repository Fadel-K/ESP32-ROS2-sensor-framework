#ifndef HEADER_H
#define HEADER_H

#include <stdint.h>

#define HEADER_SIZE_BYTES 7

typedef struct {   // header declaration
    uint64_t time_stamp_ms;
    uint8_t sensor_id;
    uint8_t node_id;
    uint8_t len;
} Header;

#endif