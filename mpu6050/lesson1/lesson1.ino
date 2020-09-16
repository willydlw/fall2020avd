/* lesson 1 - MPU6050 
      - establish I2C communication with sensor
      - Serial print used to report 
        - success or failure testing I2C communication between Arduino & sensor
        - print state of power management register 1
*/
#include <Arduino.h>
#include <Wire.h>

// MPU 6050 I2C address controlled by AD0 pin
#define MPU6050_ADDRESS_AD0_LOW     0x68   // i2c address AD0 pin low (GND), default for InvenSense
#define MPU6050_ADDRESS_AD0_HIGH    0x69   // i2c address AD0 pin high (VCC)
#define MPU6050_ADDRESS_DEFAULT     MPU_ADDRESS_AD0_LOW


// MPU 6050 register addresses
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_WHO_AM_I         0x75  

// value we expect to read from who am i register
#define EXPECTED_WHO_AM_I_RESPONSE  0x68


bool testCommunication(uint8_t devAddr)
{
  uint8_t state;
  state = readByte(devAddr, MPU6050_RA_WHO_AM_I);
  
  /*  Documentation - Who Am I register
   *   
   *   Contains the 6-bit I C address of the MPU-60X0.
       The Power-On-Reset value of Bit6:Bit1 is 110 100.

       9/16/2020 Some units are returning 0x72 instead of 0x68
   * 
   */
  if(state == EXPECTED_WHO_AM_I_RESPONSE || state == 0x72){
    Serial.print("I2C communication success, who am i returned, 0x");
    Serial.print(state, HEX);
    Serial.println("\n");
    return true;
  }
  else{
    Serial.print("error, testCommunication, state: 0x");
    Serial.println(state,HEX);
    return false;
  }
}

void reportPowerState(uint8_t devAddr)
{
  uint8_t state;
  state = readByte(devAddr, MPU6050_RA_PWR_MGMT_1);
  Serial.print("** Power Management Register 1, ");
  Serial.println(state,HEX);
  Serial.print("Bit 7, Device Reset: ");
  Serial.println(state >> 8, HEX);
  Serial.print("Bit 6, Sleep:        ");
  // 0x40  0100 0000
  Serial.println( (state & 0x40) >> 6, HEX);
  if( (state & 0x40) >> 6 == 1)
  {
    Serial.println("\t** Device is in sleep mode **\n");
  }
  Serial.print("Bit 5, Cycle:        ");
  // 0x 20 0010 0000
  Serial.println( (state & 0x20) >> 5, HEX);
  Serial.print("Bit 3, Temp_Dis      ");
  Serial.println( (state & 0x08) >> 3, HEX);
  Serial.print("Bits 2:0, CLKSEL     ");
  Serial.println( (state &0x07), HEX);
  Serial.println("");
  
}


uint8_t readByte(uint8_t devAddr, uint8_t regAddr)
{
  uint8_t data;
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  Wire.endTransmission(false);            // send TX buffer, send restart to keep connection alive
  Wire.requestFrom(devAddr, (uint8_t)1);  // request 1 byte, default true sends stop message after request,
                                          // releasing i2c bus
  data = Wire.read();
  return data;
}

int main(void)
{
  init();
  Serial.begin(9600);
  Wire.begin();                 // Initiate Wire library, join I2C bus as master
  delay(100);

  if(testCommunication(MPU6050_ADDRESS_AD0_LOW) == true)
  {
    reportPowerState(MPU6050_ADDRESS_AD0_LOW);
  }
  
  while(1){
    delay(1000);    // hang out and do nothing
  }

  return 0;
}
