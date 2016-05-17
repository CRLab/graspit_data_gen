#ifndef SENSOR_READING_H
#define SENSOR_READING_H

#include "include/matvecIO.h"
#include "include/debug.h"

struct SensorReading
{
    vec3 translation;
    Quaternion orientation;
    double force;
    // {"location": [x, y, z], "orientation"}
    void printMe() const {
        DBGA(" force: " << force <<
             " {" << translation.x() << ", " << translation.y() << ", " << translation.z() << "} " <<
             " [" << orientation.w << ", " << orientation.x << ", " << orientation.y << ", " << orientation.z << "]");
    }
};

#endif
