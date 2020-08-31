/* Serial Receive Lesson 3: read a string and transmit it back
 * 
 * Serial.readStringUntil(terminator) reads characters from the serial buffer into a String.
 * 
*/

// global variable
String msg;


void setup() {

  // default config is 8 data bits, no parity, one stop bit  8N1
  Serial.begin(38400);
  delay(100);
}

void loop() {

  if(Serial.available() > 0)
  {
    msg = Serial.readStringUntil('\n');

    // echo back the message received
    Serial.println(msg);
  }
  else
  {
    delay(50);
  }
}
