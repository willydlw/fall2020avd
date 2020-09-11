#define SERIAL_DEBUG

const unsigned long BLINK_INTERVAL = 500;
unsigned long blinkTime;
unsigned long wasteTime;
byte ledState;

void setup()
{
    #ifdef SERIAL_DEBUG
      Serial.begin(38400);
      delay(250);
      Serial.println("Serial Debug Ready");
    #endif
    pinMode(LED_BUILTIN, OUTPUT);
    ledState = 0;
    digitalWrite(LED_BUILTIN, ledState);
    blinkTime = millis();
}

void loop()
{
    if(millis() - blinkTime >= BLINK_INTERVAL)
    {
        ledState = ledState ^ 0x01;     // toggle state 
        digitalWrite(LED_BUILTIN, ledState);
        blinkTime = millis();
        #ifdef SERIAL_DEBUG
          Serial.print("ledState: ");
          Serial.println(ledState);
          Serial.flush();               // waits for transmission to complete
        #endif 
    }
    else{
      wasteTime = millis();
      while((millis() - wasteTime) < 200);
    }
}
