/*  lesson 8a - Read and transmit MPU6050 data
 *    
 *    targeted sample rate is 1 kHz
 *   
 *   Settings: 
 *    Serial baud rate 115200
 *    I2C clock frequency is standard mode 100 kHz
 */
#include <Arduino.h>
#include <Wire.h>

#define MPU6050_ADDRESS_AD0_LOW 0x68      // i2c address AD0 pin low (GND), default

#define ACCEL_XOUT_H 0x3B

#define SAMPLE_INTERVAL_USEC   1000        // 1 kHz is 1000 micro seconds


void mysetup(void)
{
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDRESS_AD0_LOW);
  Wire.write(0x6B);                       // power management 1 register
  Wire.write(0);                          // setting bit 6 to zero, wakes up device
                                          // setting other bits to zero turns off reset
                                          // and uses internal oscillator as clock
  Wire.endTransmission(true);
  Serial.println("setup complete");
  Serial.flush();                         // function does not return until data is transmitted
}


int main(void)
{
  int16_t ax, ay, az, temp, gx, gy, gz;
  unsigned long sampleTime;
  
  init();
  mysetup();
  sampleTime = micros();

  while(1)
  {
    if( (micros() - sampleTime) >= SAMPLE_INTERVAL_USEC )
    {
      
      // read data from registers
      Wire.beginTransmission(MPU6050_ADDRESS_AD0_LOW);
      Wire.write(0x3B);                     // start reading register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU6050_ADDRESS_AD0_LOW, 14);  // request reading 14 bytes
      ax = Wire.read() << 8 | Wire.read();    // reads 0X3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
      ay = Wire.read() << 8 | Wire.read(); 
      az = Wire.read() << 8 | Wire.read(); 
      temp = Wire.read() << 8 | Wire.read(); 
      gx = Wire.read() << 8 | Wire.read(); 
      gy = Wire.read() << 8 | Wire.read(); 
      gz = Wire.read() << 8 | Wire.read(); 
     
  
      Serial.print("ax: \t");   Serial.print(ax);
      Serial.print("\tay: \t"); Serial.print(ay);
      Serial.print("\taz: \t"); Serial.print(az);
      Serial.print("\ttemp: \t"); Serial.print(temp);
      Serial.print("\tgx: \t");   Serial.print(gx);
      Serial.print("\tgy: \t"); Serial.print(gy);
      Serial.print("\tgz: \t"); Serial.println(gz);
      
      sampleTime = micros();
    }
    
  }

  return 0;
  
}
