#include <Arduino.h>
#include "pins.h"
#include "motor.h"



int main(void){

  unsigned char motorSpeed = 75;
  
  init();
  Serial.begin(9600);
  initMotors();
  Serial.println("motors initiliazed, driving forward");
  driveForward(motorSpeed);
  delay(3000);
  Serial.println("backward");
  driveBackward(motorSpeed);
  delay(3000);
  stopMotors();
  Serial.println("motors stopped, end of demo");
  while(1)
  {
    delay(1000);
  }
  return 0;
}
