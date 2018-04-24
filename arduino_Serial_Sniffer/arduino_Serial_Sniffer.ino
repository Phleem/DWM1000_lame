#include <SoftwareSerial.h>

const byte rx = 2;
const byte tx = 3;

SoftwareSerial myserial(rx, tx);

void setup() {
  Serial.begin(9600);
  myserial.begin(9600);
}


void loop() {
  // put your main code here, to run repeatedly:
  if(myserial.available() > 0){
    Serial.write(myserial.read());
  }
  
  //Serial.write("HelloWorld");
  //delay(1000);
}
