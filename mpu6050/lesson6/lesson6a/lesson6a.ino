#include <Arduino.h>
#include <Wire.h>
#include "mympu6050.h"
#include "report.h"

bool mysetup(MyMPU6050& mpu)
{
  uint8_t byteRead;
  Serial.begin(38400);
  Wire.begin();                 // Initiate Wire library, join I2C bus as master
  delay(200);

  byteRead = mpu.testConnection();
  if(byteRead != 0x68)
  {
    Serial.println(F("success communicating with MPU 6050"));
    return false;
  }

  mpu.initialize();
  
  return true;
}

int main(void)
{
  MyMPU6050 mpu(MPU6050_ADDRESS_AD0_LOW);
  SensorData sd;
  uint8_t bytesRead;
  
  init();         // Arduino function, config timers and other hardware

  if(mysetup(mpu) == false){
    Serial.println("setup failed, entering infinite do-nothing while loop");
    while(1)
    {
      delay(1000);
    }
  }

  reportPowerState(mpu);
  delay(3000);
  reportConfigState(mpu);
  delay(3000);
  reportAccelConfigState(mpu);
  delay(3000);
  reportGyroConfigState(mpu);
  Serial.println("");
  reportRegisterOffsets(mpu);
  delay(5000);

  while(1)
  {
    bytesRead = mpu.readAllData(&sd);
    if(bytesRead == MPU6050_ALL_MEASUREMENT_BYTES)
    {
      printAllData(&sd);
    }
    else
    {
      Serial.print("bytes read: ");
      Serial.println(bytesRead);
    }
    delay(500);
    
  }
  
  return 0;
}
