#include "pins.h"
#include <Arduino.h>
#include <util/atomic.h>

// Global Constants
const unsigned long ENCODER_SAMPLE_INTERVAL = 500UL;    // units, milliseconds

// Global Variables
volatile unsigned long leftEncoderCount = 0;


/* Interrupt Service Routine
 *  updates left encoder count
 */
void leftEncoderISR(void)
{
  leftEncoderCount++;
}


void mysetup(void)
{
  pinMode(LEFT_ENCODER_A_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A_PIN), leftEncoderISR, CHANGE);
}

int main(void)
{
  unsigned long startTime, dTEncoder;
  unsigned long tempLeftEncoderCount;
  
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
        leftEncoderCount = 0UL;       // 0 is type int, 0UL is type unsigned long
      }
      /* or instead of the ATOMIC_BLOCK, turn off interrupts
     
      noInterrupts();               // disable interrupts
      tempLeftEncoderCount = leftEncoderCount;
      leftEncoderCount = 0UL;
      interrupts();                 // re-enable interrupts

      */
      
    }
  }

  return 0;
}
