#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;
/*int counter;

union TransferBytes{
    struct bytes{
      uint8_t high;
      uint8_t low;
    };
    uint16_t val;
};

TransferBytes transferBytes;
*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);//changed the baud from 9600 to 115200 as seen in the example sensor code
  Wire.begin();
  Wire.setClock(400000);
  //counter = 1;
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    //Serial.println("It no worky");
    //while(1);
    while(1) Serial.write(4);
  }

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(50);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensor.read();
  float floatInches = (float)(sensor.ranging_data.range_mm * 0.0393701);
  floatInches = floatInches * 2.0;
  int intInches = (int)(floatInches);
  //if(dataReady()) Serial.write(dataInches);
  Serial.write(intInches);
  //Serial.println(dataInches);
  /*
  transferBytes.val = sensor.ranging_data.range_mm;
  
  if(counter==1){
    Serial.write(transferBytes.bytes.high);
    counter=2;
  } else if(counter==2) {
    Serial.write(transferBytes.bytes.low);
    counter=0;
  } else {
    uint8_t test1 = transferBytes.bytes.high;
    uint8_t test2 = transferBytes.bytes.low;
    Serial.write(test1 & test2);
    counter=1;
}
*/
  
}
