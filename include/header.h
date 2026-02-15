#include <stdint.h>

struct Header {   // header declaration
    uint64_t time_stamp;
    uint8_t sensor_id;
    uint8_t node_id;
    uint8_t len;
}; 