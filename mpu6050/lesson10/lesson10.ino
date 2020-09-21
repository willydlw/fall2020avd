/*  Lesson 10: Calculating Angular Orientation with Gyroscope Data
 *   
 *   Tilt sensing of roll, pitch, yaw angle from gryo data
 *   
 *   
 *  Setup
 *    MPU-6050 - default configuration 
 *    i2c clock frequency 400000
 *    Serial baud rate 115200
 *  
 *  calculates gyro offset values
 *  
 */
#include <Arduino.h>
#include <Wire.h>
#include "mpu6050.h"
#include "helper.h"


//#define DEBUG                     // turn on for all debug info
//#define PRINT_ALL_GYRO              // prints all gyro related info
//#define PRINT_POSITION              // prints gyro position estimate, summed over time  
#define PLOT_POSITION             // turn on for serial plotter

#define SAMPLE_INTERVAL_MS  4           // milliseconds


// Initialize Serial, I2C, and mpu6050
void setup(void)
{
  Wire.begin();
  Wire.setClock(400000L);
  Serial.begin(115200);
  delay(100);
  setupMPU6050();
  
  #ifdef DEBUG
    Serial.println(F("setup complete"));
  #endif
}



int main(void)
{
  SensorData offset, gyroRate;
  AngleData gyroPosition = {0.0, 0.0, 0.0};
  AngleData deltaPosition;                        // change in position
  unsigned long startTime, elapsedTimems;

  int sampleCount = 0;

   
  init();                           // Arduino function to initialize timers, pwm, other hardware
  setup();
  
  #ifdef DEBUG
    Serial.println(F("ready to calibrate"));
  #endif
 
  calibrateGyro(&offset, 10);

  #ifdef DEBUG
    Serial.println(F("calibration complete"));
    printOffsetValues(&offset);
    delay(1000);
  #endif
  
  startTime = millis();
  while(1)
  {
    if( (elapsedTimems = (millis() - startTime)) >= SAMPLE_INTERVAL_MS)
    {
      readGyro(&gyroRate);
      startTime = millis(); 
      ++sampleCount;

      #ifdef DEBUG
        Serial.println("before offset");
        printSensorData(&gyroRate);
      #endif
        
      gyroRate.x += offset.x;          // apply offset
      gyroRate.y += offset.y;
      gyroRate.z += offset.z;

      #ifdef DEBUG
        Serial.println("after offset");
        printSensorData(&gyroRate);
      #endif



      #ifdef PRINT_ALL_GYRO
        Serial.println("gyro rate, raw data");
        printSensorData(&gyroRate);
      #endif

      // scale the data to deg/s and calculate change in position
      deltaPosition.x = (double)gyroRate.x / 131.0 * elapsedTimems / 1000.0;
      deltaPosition.y = (double)gyroRate.y / 131.0 * elapsedTimems / 1000.0;
      deltaPosition.z = (double)gyroRate.z / 131.0 * elapsedTimems / 1000.0;

      #ifdef PRINT_ALL_GYRO
        Serial.println("change in position, deg");
        printAngleData(&deltaPosition, 4);
      #endif

      gyroPosition.x += deltaPosition.x;
      gyroPosition.y += deltaPosition.y;
      gyroPosition.z += deltaPosition.z;


      #ifdef PRINT_ALL_GYRO
        Serial.println("gyro position, deg");
        printAngleData(&gyroPosition,4);
      #endif
     
      #ifdef PRINT_POSITION
        if( sampleCount % 100 == 0 )
        {
          Serial.println("gyro position, deg");
          printAngleData(&gyroPosition,4);
          sampleCount = 0;
        }
      #endif

      #ifdef PLOT_POSITION
        Serial.print(gyroPosition.x);
        Serial.print(",");
        Serial.print(gyroPosition.y);
        Serial.print(",");
        Serial.println(gyroPosition.z);
      #endif
    }
  }
    
   
  return 0;
  
}
