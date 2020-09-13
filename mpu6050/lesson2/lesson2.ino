#include <Arduino.h>
#include <Wire.h>

// MPU 6050 I2C address
#define MPU6050_ADDRESS_AD0_LOW     0x68   // i2c address pin low (GND), default
#define MPU6050_RA_CONFIG       0x1A
#define MPU6050_RA_GYRO_CONFIG  0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C

// MPU 6050 register addresses
#define MPU6050_RA_PWR_MGMT_1   0x6B
#define MPU6050_RA_WHO_AM_I     0x75  

// MPU 6050 Clock Settings
#define MPU6050_CLOCK_PLL_XGYRO 0x01

// MPU gyro full scale range settings 
#define MPU6050_GYRO_FS_250  0x00
#define MPU6050_GYRO_FS_500  0x01
#define MPU6050_GYRO_FS_1000 0x02
#define MPU6050_GYRO_FS_2000 0x03

// MPU accel full scale settings 
#define MPU6050_ACCEL_FS_2G  0x00
#define MPU6050_ACCEL_FS_4G  0x01
#define MPU6050_ACCEL_FS_8G  0x02
#define MPU6050_ACCEL_FS_16G 0x03


bool testCommunication(uint8_t devAddr)
{
  uint8_t state;
  state = readByte(devAddr, MPU6050_RA_WHO_AM_I);
  if(state == 0x68){
    Serial.println("success, testCommunication\n");
    return true;
  }
  else{
    Serial.print("error, testCommunication, expected: 0x68, regVal: ");
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
  Serial.println( (state & 0x40) >> 6, HEX);
  if( (state & 0x40) >> 6 == 1)
  {
    Serial.println("\t** Device is in sleep mode **\n");
  }
  Serial.print("Bit 5, Cycle:        ");
  Serial.println( (state & 0x20) >> 5, HEX);
  Serial.print("Bit 3, Temp_Dis      ");
  Serial.println( (state & 0x08) >> 3, HEX);
  Serial.print("Bits 2:0, CLKSEL     ");
  Serial.println( (state &0x07), HEX);
  Serial.println("");
  
}

void reportConfigState(uint8_t devAddr)
{
  uint8_t state;
  state = readByte(devAddr, MPU6050_RA_CONFIG);
  
  Serial.println("\n*** Configuration ***");
  if( ((state & 0x38) >> 3) == 0)
  {
    Serial.println("External Frame Synchronization Disabled as expected");
  }
  else
  {
    Serial.print("Warning: external frame synchronization enabled, setting: ");
    Serial.println((state & 0x38) >> 3, HEX);
  }

  Serial.println("\nDigital Low Pass Filter Configuration");
  Serial.println("  Accelerometer              Gyroscope");
  Serial.println("   (Fs = 1 kHz)");
  Serial.println("Bandwidth  Delay   Bandwidth  Delay  Fs(kHz)");
  Serial.println("(Hz)       (ms)    (Hz)       (ms)");
  Serial.println("============================================");
  switch(state&0x07)
  {
    case 0:
      Serial.println("    260    0       256        0.98      8");
    break;
    case 1:
      Serial.println("    184    2.0     188        1.9       1");
    break;
    case 2:
      Serial.println("     94    3.0      98        2.8       1");
    break;
    case 3:
      Serial.println("     44    4.9      42        4.8       1");
    break;
    case 4:
      Serial.println("     21    8.5      20        8.3       1");
    break;
    case 5:
      Serial.println("     10   13.8      10       13.4       1");
    break;
    case 6:
      Serial.println("      5   19.0       5       18.6       1");
    break;
    default:
      Serial.print("unknown, value: "); Serial.println(state&0x07, HEX);
  }

  Serial.println("");
}

void reportAccelConfigState(uint8_t devAddr)
{
  uint8_t state;
  state = readByte(devAddr, MPU6050_RA_ACCEL_CONFIG);
  Serial.print("\n*** Accel Config, ");
  Serial.println(state, HEX);
  if( (state & 0xE0) == 0)
  {
    Serial.println("self-test disabled");
  }
  else
  {
    Serial.print("self-test enabled, bits 7:5: ");
    Serial.println((state & 0xE0) >> 5, HEX);
  }

  Serial.print("full scale range, +-");
  switch( (state & 0x18) >> 3)
  {
    case MPU6050_ACCEL_FS_2G:
      Serial.print(2);
    break;
    case MPU6050_ACCEL_FS_4G:
      Serial.print(4);
    break;
    case MPU6050_ACCEL_FS_8G:
      Serial.print(8);
    break;
    case MPU6050_ACCEL_FS_16G:
      Serial.print(16);
    break;
  } 
  Serial.println(" g");
}

void reportGyroConfigState(uint8_t devAddr)
{
  uint8_t state;
  state = readByte(devAddr, MPU6050_RA_GYRO_CONFIG);
  Serial.print("\n*** Gyro Config, ");
  Serial.println(state, HEX);
  if( (state & 0xE0) == 0)
  {
    Serial.println("self-test disabled");
  }
  else
  {
    Serial.print("self-test enabled, bits 7:5: ");
    Serial.println((state & 0xE0) >> 5, HEX);
  }

  Serial.print("full scale range, +-");
  switch( (state & 0x18) >> 3)
  {
    case MPU6050_GYRO_FS_250:
      Serial.print(250);
    break;
    case MPU6050_GYRO_FS_500:
      Serial.print(500);
    break;
    case MPU6050_GYRO_FS_1000:
      Serial.print(1000);
    break;
    case MPU6050_GYRO_FS_2000:
      Serial.print(2000);
    break;
  } 

  Serial.println(" deg/s");
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
  Serial.begin(38400);
  Wire.begin();                 // Initiate Wire library, join I2C bus as master
  delay(100);

  if(testCommunication(MPU6050_ADDRESS_AD0_LOW) == true)
  {
    reportPowerState(MPU6050_ADDRESS_AD0_LOW);
    reportConfigState(MPU6050_ADDRESS_AD0_LOW);
    reportGyroConfigState(MPU6050_ADDRESS_AD0_LOW);
    reportAccelConfigState(MPU6050_ADDRESS_AD0_LOW);
  }

  while(1);
  return 0;
}
