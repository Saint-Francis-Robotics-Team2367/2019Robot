#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);//changed the baud from 9600 to 115200 as seen in the example sensor code
  Wire.begin();
  Wire.setClock(400000);

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    while(1) Serial.write(0);
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
  int dataInches = (int)(sensor.ranging_data.range_mm * 0.0393701);
  //Serial.write(dataInches);
  Serial.println(dataInches);
}
