#ifndef HELPER_H_LESSON9
#define HELPER_H_LESSON9

#include "mpu6050.h"          // SensorData


void printOffsetValues(const SensorData* offset);
void printAccelValues(const SensorData* accel);
void printAngles(double roll, double pitch);


#endif
