#include <Servo.h>
Servo servo1,servo2;
void setup() {
  // put your setup code here, to run once:
servo1.attach(11);
servo2.attach(8);
servo1.write(105);
servo2.write(85);
}

void loop() {
  // put your main code here, to run repeatedly:

}
