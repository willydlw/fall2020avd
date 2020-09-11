#include <Arduino.h>

double waitForFrequency(void)
{
  double freq;
  bool success = false;
  while(success == false)
  {
    while(Serial.available() < (int)sizeof(double)){
      // do nothing
    }
    if(Serial.readBytes((char*)&freq, sizeof(freq)) == sizeof(freq)){
      Serial.println("ACK");
      success = true;
    }
    else{
      Serial.println("NAK");
    }
  }

  return freq;
  
}

/*
double waitForFrequency(void)
{
  byte data[4];
  size_t bytesRead;
  int count = 0;
  while(count < 4)
  {
    if(Serial.available() > 0)
    {
      bytesRead = Serial.readBytes(data,1);
      if(bytesRead == 1){
        Serial.write(data[count]);
        ++count;
      }
      else{
        Serial.println("no bytes read");
      }
    }
  }

  
  
  return 5.0;
}
*/

// declare reset function at address 0



void(* resetFunc)(void)=0;    

int main()
{
  init();
  Serial.begin(9600);
  delay(1000);
  double freq = waitForFrequency();
  Serial.println(freq);

  while(1)
  {
    Serial.println("Arduino main's infinite loop");
    delay(2000);
    resetFunc();
  }
  return 0;
}
