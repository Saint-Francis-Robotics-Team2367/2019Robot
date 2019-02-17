#include <Wire.h>
/*uint8_t x = 0;
void setup() {
  // Start the I2C Bus as Master
  Wire.begin(4);
  Wire.onRequest(requestEvent);
}
void loop() {
  //Wire.beginTransmission(9); // transmit to device #9
  //Wire.write(x);              // sends x 
  //Wire.endTransmission();    // stop transmitting
  x++; // Increment x
  if (x > 5) x = 0; // `reset x once it gets 6
  delay(500);
}

void requestEvent(){
  Wire.write(x);
}
*/
String output;
void setup()
{
  Wire.begin(4);                // join i2c bus with address #4
  //Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // a request event
  Serial.begin(9600);
}

void loop()
{
 
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent()
{
  /*uint8_t myTestInteger3 = 65;
  //char ourChar = 'a';
  while ( Wire.available() > 0 )
  {
    //uint8_t n=Wire.read();
    //Serial.println(n);
    Wire.write(myTestInteger3);
  }*/
  
}

void requestEvent(){
  output = "test";
  uint8_t myTestInteger3 = 65;
  while ( Wire.available() > 0 )
  {
    Wire.write(output.c_str());
  }
}
