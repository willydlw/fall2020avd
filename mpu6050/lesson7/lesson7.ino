/*  Read and transmit accelerometer data from MPU-6050
 * 
 *  Serial transmission: accelerometer x, y, z data type is int16_t, range -32768, +32767. 
 *   
 *  Sampling interval is 20 milliseconds, .020 seconds
 *  
 *  Make sure the baud rate chosen is fast enough to transmit the data at the 
 *  sample rate.
 */


#include <Arduino.h>
#include <Wire.h>

const int MPUADDRESS = 0x68;



void setup() {
  Wire.begin();                   // Wire library initialization
  Wire.beginTransmission(MPUADDRESS);   // Begin transmission to MPU, default I2C address
  Wire.write(0x6B);               // PWR_MGMT_1 register
  Wire.write(0);                  // MPU-6050 to start mode
  Wire.endTransmission(true);
  Serial.begin(38400);
  delay(100);
}

void readAccel(int16_t& x, int16_t& y, int16_t& z)
{
  Wire.beginTransmission(MPUADDRESS);     // Start transfer
  Wire.write(0x3B);                       // register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);            // Maintain connection
  Wire.requestFrom(MPUADDRESS, 6, true);         
  
  x = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  y = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  z = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
}

void sendData(int16_t x, int16_t y, int16_t z)
{
  Serial.print(x);
  Serial.print(","); 
  Serial.print(y);
  Serial.print(","); 
  Serial.println(z);
}

int main(void)
{
  const unsigned long SAMPLE_INTERVAL = 20;     // milliseconds
  
  int16_t x, y, z;
  unsigned long startTime;

  init();
  setup();

  startTime = millis();
  
  while(1)
  {
    if( (millis() - startTime) >= SAMPLE_INTERVAL)
    {
      readAccel(x, y, z);
      sendData(x, y, z);
      startTime = millis();
    }
  }

  return 0;
  
}
