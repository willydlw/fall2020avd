#include "helper.h"

#include <Arduino.h>          // Serial

void printOffsetValues(const SensorData* offset)
{
  Serial.print("offset values, x: ");
  Serial.print(offset->x);
  Serial.print(", y: ");
  Serial.print(offset->y);
  Serial.print(", z: ");
  Serial.println(offset->z);
}

void printAccelValues(const SensorData* accel)
{
  Serial.print("ax: \t");   Serial.print(accel->x);
  Serial.print("\tay: \t"); Serial.print(accel->y);
  Serial.print("\taz: \t"); Serial.println(accel->z);
}


void printAngles(double roll, double pitch)
{
  Serial.print("roll: ");
  Serial.print(roll);
  Serial.print("\tpitch: ");
  Serial.println(pitch);
}
