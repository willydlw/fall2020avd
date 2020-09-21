/*  lesson 8c - Read and transmit data at a sample rate of 1 kHz
 *  
 *  Questions:
 *    How much time does it take
 *      - read MPU6050 measurement registers?
 *      - serial transmit data using write
 *  
 *  Calculates average time to read data registers and transmit
 *  using Serial.write
 *   
 */
#include <Arduino.h>
#include <Wire.h>

#define MPU6050_ADDRESS_AD0_LOW 0x68      // i2c address AD0 pin low (GND), default

#define ACCEL_XOUT_H 0x3B

#define SAMPLE_INTERVAL_USEC   1000        // 1 kHz is 100 micro seconds


void mysetup(void)
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000L);
  delay(100);
  Wire.beginTransmission(MPU6050_ADDRESS_AD0_LOW);
  Wire.write(0x6B);                       // power management 1 register
  Wire.write(0);                          // setting bit 6 to zero, wakes up device
                                          // setting other bits to zero turns off reset
                                          // and uses internal oscillator as clock
  Wire.endTransmission(true);
  Serial.println("setup complete");
  delay(250);
}


int main(void)
{
  int16_t data[7] = { 1, 2, 3, 4, 5, 6, 7};
  
  unsigned long startTime, stopTime, sampleTime;
  unsigned long avgReadTime = 0, avgTxTime = 0;
  int numSamples = 5;
  int count = 0;

  
  init();
  mysetup();
  sampleTime = micros();

  Serial.println("Illustrate byte pointer correctly accesses array contents");
  Serial.write((byte*)data, 14);
  Serial.flush();
  delay(1000);
  

  while(1)
  {
    if(count < numSamples && (micros() - sampleTime) >= SAMPLE_INTERVAL_USEC)
    {
      startTime = micros();
      // read data from registers
      Wire.beginTransmission(MPU6050_ADDRESS_AD0_LOW);
      Wire.write(0x3B);                     // start reading register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU6050_ADDRESS_AD0_LOW, 14);  // request reading 14 bytes
      data[0] = Wire.read() << 8 | Wire.read();    // reads 0X3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
      data[1] = Wire.read() << 8 | Wire.read(); 
      data[2] = Wire.read() << 8 | Wire.read(); 
      data[3] = Wire.read() << 8 | Wire.read(); 
      data[4] = Wire.read() << 8 | Wire.read(); 
      data[5] = Wire.read() << 8 | Wire.read(); 
      data[6] = Wire.read() << 8 | Wire.read(); 
      stopTime = micros();

      avgReadTime += stopTime - startTime;
  
  
      Serial.print("\nelapsed read time: ");
      Serial.print(stopTime - startTime);
      Serial.println(" usec");
      Serial.flush();                         // Waits for the transmission of outgoing serial data to complete.
  
  
      startTime = micros();
      Serial.write( (byte*)data, 14);         // data is int16_t*
      //Serial.flush();
      stopTime = micros();

      avgTxTime += stopTime - startTime;
  
      Serial.print("\nelapsed serial write time: ");
      Serial.print(stopTime - startTime);
      Serial.println(" usec");
      //Serial.flush();

      ++count;
      sampleTime = micros();
    }
    else if (count == numSamples)
    {
      ++count;
      avgReadTime /= numSamples;
      avgTxTime /= numSamples;
      Serial.print("\naverage read time: ");
      Serial.print(avgReadTime);
      Serial.print(" usec, average tx time: ");
      Serial.print(avgTxTime);
      Serial.println(" usec");
    }
  }

  return 0;
  
}
