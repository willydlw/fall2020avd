/*  lesson 8b - Read and transmit data at a sample rate of 1 kHz
 *  
 *  Questions:
 *    How much time does it take
 *      - read MPU6050 measurement registers?
 *      - serial transmit data using print
 *      
 *  Calculates average time to read data registers and transmit using
 *  Serial.print
 *  
 *  Settings: 
 *    Serial baud rate 115200
 *    I2C clock frequency is standard mode 100 kHz
 *   
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
  Wire.setClock(400000L);
  delay(100);
  Wire.beginTransmission(MPU6050_ADDRESS_AD0_LOW);
  Wire.write(0x6B);                       // power management 1 register
  Wire.write(0);                          // setting bit 6 to zero, wakes up device
                                          // setting other bits to zero turns off reset
                                          // and uses internal oscillator as clock
  Wire.endTransmission(true);
  Serial.println("setup complete");
}


int main(void)
{
  int16_t ax, ay, az, temp, gx, gy, gz;
  unsigned long startTime, stopTime, sampleTime;
  unsigned long avgReadTime = 0, avgTxTime = 0;
  int numSamples = 5;
  int count = 0;

  
  init();
  mysetup();
  sampleTime = micros();

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
      ax = Wire.read() << 8 | Wire.read();    // reads 0X3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
      ay = Wire.read() << 8 | Wire.read(); 
      az = Wire.read() << 8 | Wire.read(); 
      temp = Wire.read() << 8 | Wire.read(); 
      gx = Wire.read() << 8 | Wire.read(); 
      gy = Wire.read() << 8 | Wire.read(); 
      gz = Wire.read() << 8 | Wire.read(); 
      stopTime = micros();

      avgReadTime += stopTime - startTime;
  
  
      Serial.print("elapsed read time: ");
      Serial.print(stopTime - startTime);
      Serial.println(" usec");
      Serial.flush();                         // Waits for the transmission of outgoing serial data to complete.
  
  
      startTime = micros();
      /*
      Serial.print("ax: \t");   Serial.print(ax);
      Serial.print("\tay: \t"); Serial.print(ay);
      Serial.print("\taz: \t"); Serial.print(az);
      Serial.print("\ttemp: \t"); Serial.print(temp);
      Serial.print("\tgx: \t");   Serial.print(gx);
      Serial.print("\tgy: \t"); Serial.print(gy);
      Serial.print("\tgz: \t"); Serial.println(gz);
      */

      // faster transmit by removing labels, tabs
      Serial.print(ax);
      Serial.print(",");
      Serial.print(ay);
      Serial.print(",");
      Serial.print(az);
      Serial.print(",");
      Serial.print(temp);
      Serial.print(",");
      Serial.print(gx);
      Serial.print(",");
      Serial.print(gy);
      Serial.print(",");
      Serial.println(gz);
      
       
      //Serial.flush();
      stopTime = micros();

      avgTxTime += stopTime - startTime;
  
      Serial.print("elapsed serial print time: ");
      Serial.print(stopTime - startTime);
      Serial.println(" usec");
      Serial.flush();

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
