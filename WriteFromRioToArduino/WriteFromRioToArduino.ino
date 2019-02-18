#include <Wire.h>
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin(4);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void receiveEvent(int bytes){
  while (Wire.available()) {
    uint8_t c = Wire.read();
    Serial.print(c);
  }
  Serial.println("");
}

void requestEvent(){
  uint8_t e = 2;
  char d = 'd';
  String f = "hello";
  String mm = "    " + 3;
  int g = 111;
  int h = (int)1;
  Wire.write(g);
}
