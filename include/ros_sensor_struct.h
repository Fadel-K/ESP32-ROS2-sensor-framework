#ifndef ROS_SENSOR_STRUCT_H
#define ROS_SENSOR_STRUCT_H

#include "header.h"

typedef struct {
    Header header;
    uint8_t data[];
} GeneralSensor;

  // unpack to an 8 byte array CAN payload
void pack_sensor_to_array(const GeneralSensor *sensor, uint8_t* buffer) { //TODO: Could implement checking if buffer is too small
    uint8_t i=0;
    
    //converting header into 8 bit array (timestamp is 64-bit)
    uint64_t time_stamp = sensor->header.time_stamp_ms;
    buffer[i++] = (uint8_t)(time_stamp & 0xFF);
    buffer[i++] = (uint8_t)((time_stamp >> 8) & 0xFF);
    buffer[i++] = (uint8_t)((time_stamp >> 16) & 0xFF);
    buffer[i++] = (uint8_t)((time_stamp >> 24) & 0xFF);
    buffer[i++] = (uint8_t)((time_stamp >> 32) & 0xFF);
    buffer[i++] = (uint8_t)((time_stamp >> 40) & 0xFF);
    buffer[i++] = (uint8_t)((time_stamp >> 48) & 0xFF);
    buffer[i++] = (uint8_t)((time_stamp >> 56) & 0xFF);

    buffer[i++]=sensor->header.sensor_id;
    buffer[i++]=sensor->header.node_id;
    buffer[i++]=sensor->header.len;

    // do {buffer[i]=i; i++;} while (i<16);
    //adding data into 8 bit array
    memcpy(&buffer[i], sensor->data, sensor->header.len);
}

#endif