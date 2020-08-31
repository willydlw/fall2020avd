/* Serial Transmit Example 2a: send data type byte, unsigned 8 bit
 * with write command.
 * 
 * Serial.write  writes binary data to the serial port. 
 * 
 * Serial.write(val)  val is value to send as a single byte
 * 
 * Example: The value 1   is transmitted as 00000001
 *          The value 9   is transmitted as 00001001
 *          The value 32  is transmitted as 00100000
 *          The value 255 is tranmsitted as 11111111
 * 
*/

// global variable
byte count;             // 0 - 255


void setup() {

  // default config is 8 data bits, no parity, one stop bit  8N1
  Serial.begin(9600);
  count = 253;
  delay(100);
}

void loop() {
  Serial.write(count);
  ++count;
  delay(500);
}
