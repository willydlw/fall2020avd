# Arduino - Multiple Pin Change Interrupts in a Port

How do we know which pin caused an interrupt when multiple pin change interrupts are enabled in the same port?

In the following example, Arduino pins D12 and D11 will both be enabled as pin change interrupts. Both of these pins are in port B. The program below illustrates how to detect which pin caused the interrupt.</br></br>

```cpp
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <Arduino.h>

volatile uint8_t portBHistory = 0xFF;
volatile uint8_t count12 = 0;
volatile uint8_t count11 = 0;

void mysetup(void){
   pinMode(12, INPUT_PULLUP);
   pinMode(11, INPUT_PULLUP);
   cli();
   // Enables Port B Pin Change Interrupts
   PCICR |= 0b00000001;    // alternative code: PCICR |= (1 << PCIE0); 

   // enable PCINT4, Arduino D12, Port B, pin PB4
   // enable PCINT3, Arduino D11, Port B, pin PB3 
   PCMSK0 |= 0b0001100;   // alternative code: PCMSK0 |= (1 <<PCINT4); 
   sei();
   
   Serial.begin(9600);
}

int main(void){
   unsigned long start = millis();
   init();
   mysetup();

   if(millis() - start > 500){
      start = millis();
      tempCount11 = count11;
      tempCount12 = count12;
      Serial.print('count 11: ');
      Serial.print(tempCount11);
      Serial.print(', count 12: ');
      Serial.println(tempCount12);
   }

   return 1;
}

ISR(PCINT0_vect)
{
  uint8_t changedBits;

  changedBits = PINB ^ portBHistory;      // xor marks changed bits with a 1
  portHistory = PINB;

  if(changedBits & (1 << PINB3)){         // pin D11, PB3 caused the interrupt
     count11++;
  }
  else if(changedBits & (1 << PINB4)){    // pin D12, PB4 caused the interrupt
     count12++;
  }
}
```
