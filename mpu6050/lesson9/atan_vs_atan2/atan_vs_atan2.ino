// atan versus atan2 for quadrant resolution

void printAngle(double x, double y)
{
  double angle1, angle2;
  angle1 = atan(y/x) * RAD_TO_DEG;
  angle2 = atan2(y,x) * RAD_TO_DEG;
  Serial.print("x: ");
  Serial.print(x);
  Serial.print(", y: ");
  Serial.print(y);
  Serial.print(", atan: ");
  Serial.print(angle1);
  Serial.print(", atan2: ");
  Serial.println(angle2);
}


void setup() {
  Serial.begin(9600);
  delay(100);
  printAngle(1.0, 1.0);         // quadrant 1
  printAngle(-1.0, 1.0);        // quadrant 2
  printAngle(-1.0, -1.0);       // quadrant 3
  printAngle(1.0, -1.0);        // quadrant 4
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
