/*
 * Arduino serially transmits a ready signal to the python program, once per second. 
 * It continues transmitting the ready signal until an acknowledgement from the 
 * python program is received.
 */
 
#include <Arduino.h>

void readyState()
{
  const unsigned long transmitInterval = 1e6;        // 1 second in units of microseconds
  const char* readyMessage = "READY";
  const char* ackMessage = "ACK";
  
  char receiveBuffer[4] = {'\0'};
  size_t bytesReceived;
  
  bool ackReceived = false;
  unsigned long start = micros();
  
  while(ackReceived == false)
  {
    if(micros() - start >= transmitInterval)
    {
      Serial.println(readyMessage);
      Serial.flush();
      start = micros();
    }
    if(Serial.available() >= (int)strlen(ackMessage))
    {
      bytesReceived = Serial.readBytesUntil('\n',receiveBuffer, 3);
      if(strcmp(ackMessage,receiveBuffer) == 0)
      {
        ackReceived = true;
      }
      else{
        Serial.print("DEBUG: readyState, bytes received: ");
        Serial.println(bytesReceived);
        Serial.print("message received: ");
        Serial.println(receiveBuffer);
      }
    }
  }
}


int main(void)
{
  init();
  Serial.begin(9600);
  readyState();
  while(1)
  {
    Serial.println("Arduino main, ready state complete");
    delay(3000);
  }
  return 0;
}
