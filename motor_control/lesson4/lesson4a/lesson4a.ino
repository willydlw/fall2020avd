#include "pins.h"
#include <Arduino.h>
#include <util/atomic.h>

// Global Constants
const unsigned long ENCODER_SAMPLE_INTERVAL = 500UL;    // units, milliseconds

// Global Variables
volatile unsigned long leftEncoderCount = 0;
volatile unsigned long rightEncoderCount = 0;



/* Interrupt Service Routine
 *  updates left encoder count
 */
void leftEncoderISR(void)
{
  leftEncoderCount++;
}

/* Interrupt Service Routine
 *  updates left encoder count
 */
void rightEncoderISR(void)
{
  rightEncoderCount++;
}

void mysetup(void)
{
  pinMode(LEFT_ENCODER_A_PIN, INPUT);
  pinMode(RIGHT_ENCODER_A_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A_PIN), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A_PIN), rightEncoderISR, CHANGE);
}

int main(void)
{
  unsigned long startTime, dTEncoder;
  unsigned long tempLeftEncoderCount, tempRightEncoderCount;
  
  init();
  mysetup(); 
  
  startTime = millis();
  while(1)
  {
    if( (millis() - startTime) >=  ENCODER_SAMPLE_INTERVAL)
    {
      dTEncoder = millis() - startTime;
      startTime = millis();
      
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // code with interrupts blocked (consecutive atomic operations will not get interrupted)
        tempLeftEncoderCount = leftEncoderCount;
        tempRightEncoderCount = rightEncoderCount;
        leftEncoderCount = 0UL;       // 0 is type int, 0UL is type unsigned long
        rightEncoderCount = 0UL;
      }
    }
  }

  return 0;
}
