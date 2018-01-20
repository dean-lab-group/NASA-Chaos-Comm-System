#include <SoftwareSerial.h>
#define RX_pin 10
#define TX_pin 11
SoftwareSerial mySerial(RX_pin, TX_pin); // RX, TX

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Arduino USB Serial is connected!");
  mySerial.begin(9600);
  mySerial.println("Arduino Softserial on (RX_pin,TX_pin)"); 
}

void loop() {
//  if (Serial.available()) {      // If anything comes in Serial (USB),
//    mySerial.write(Serial.read());   // read it and send it out mySerial (pins 10 & 11)
//  }

  if (mySerial.available()) {     // If anything comes in mySerial (pins 10 & 11)
    Serial.write(mySerial.read());   // read it and send it out Serial (USB)
  }
}
