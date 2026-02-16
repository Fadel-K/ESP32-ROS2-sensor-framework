#ifndef ROS_SENSOR_STRUCT_H
#define ROS_SENSOR_STRUCT_H

#include "header.h"

typedef struct {
    Header header;
    uint8_t data[];
} GeneralSensor;

#endif