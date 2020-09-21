/*  Tilt sensing of roll, pitch angle from accelerometer data
 *   
 *  Setup
 *    MPU-6050 - default configuration 
 *    i2c clock frequency 400000
 *    Serial baud rate 115200
 *  
 *  calculates accel offset values
 *  
 */
#include <Arduino.h>
#include <Wire.h>
#include "mpu6050.h"
#include "helper.h"


//#define DEBUG   
#define PLOT            // turn on for serial plotter

#define SAMPLE_INTERVAL_MS  50           // milliseconds


// Initialize Serial, I2C, and mpu6050
void setup(void)
{
  Wire.begin();
  Wire.setClock(400000L);
  Serial.begin(115200);
  delay(100);
  setupMPU6050();
  
  #ifdef DEGUG
    Serial.println(F("setup complete"));
  #endif
}



int main(void)
{
  SensorData offset, accel;
  unsigned long startTime;

  double roll, pitch;               // roll rotation about x, pitch rotation about y
  double rollF = 0.0;               // filtered roll
  double pitchF = 0.0; 
  
  
  init();                           // Arduino function to initialize timers, pwm, other hardware
  setup();
  
  #ifdef DEBUG
    Serial.println(F("\n\nready to calibrate\n"));
  #endif
 
  calibrateAccelerometer(&offset, 10);

  #ifdef DEBUG
    Serial.println(F("\ncalibration complete\n"));
    printOffsetValues(&offset);
    delay(3000);
  #endif
  
  startTime = millis();
  while(1)
  {
    if( (millis() - startTime) >= SAMPLE_INTERVAL_MS)
    {
      readAccelerometer(&accel);
      startTime = millis(); 

      #ifdef DEBUG
        Serial.println("before offset");
        printAccelValues(&accel);
      #endif
      
      accel.x += offset.x;          // apply offset
      accel.y += offset.y;
      accel.z += offset.z;

      #ifdef DEBUG
        Serial.println("after offset");
        printAccelValues(&accel);
      #endif

      // Freescale, equations 25 and 26, Rotation order Rxyz
      roll = atan(accel.y / accel.z) * 180.0/PI;
      pitch = atan(-accel.x / sqrt(pow(accel.y,2) + pow(accel.z,2))) * 180.0/PI;
      
      #ifdef DEBUG
        printAngles(roll, pitch);
      #endif

      
      // apply low pass filter
      rollF = 0.94 * rollF + 0.06 * roll;
      pitchF = 0.94 * pitchF + 0.06 * pitch;
      
      #ifdef PLOT
        Serial.print(roll);
        Serial.print(",");
        Serial.println(rollF);
      #endif
      
    }
  }
    
   
  return 0;
  
}
