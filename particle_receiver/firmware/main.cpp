#include "Particle.h"
SYSTEM_MODE(MANUAL);

void setup() // Put setup code here to run once
{
    Serial.begin(9600);
    Serial.println("I am the USB Serial Port");
    Serial1.begin(9600);
    Serial1.println("I am the HW Serial Port");
}

void loop() // Put code here to loop forever
{
    // read from port 0, send to port 1:
  if (Serial.available())
  {
    char letter = Serial.read();
    Serial1.printf("%d\n0\n%d\n0\n%d\n0\n0\n", letter, letter, letter);
    // Serial1.write(letter);
  }
  // read from port 1, send to port 0:
  if (Serial1.available())
  {
    int inByte = Serial1.read();
    Serial.write(inByte);
  }
}
