#include <Wire.h>
#include <VL53L1X.h>

VL53L1X leftSensor;
VL53L1X rightSensor;

int counter;
bool functioningLeft;
bool functioningRight;

void setup() {
  
  Serial.begin(9600);//changed the baud from 9600 to 115200 as seen in the example sensor code
  Wire.begin();
  Wire.setClock(400000);

  counter = 0;
  functioningLeft = true;
  functioningRight = true;

  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  
  digitalWrite(2, LOW);//turn off right sensor
  digitalWrite(4, HIGH);//turn on left sensor
  
  leftSensor.setTimeout(500);
  rightSensor.setTimeout(500);
  
  if(!leftSensor.init()){
    functioningLeft = false;
  } else {
    leftSensor.setAddress(1);
    leftSensor.setDistanceMode(VL53L1X::Medium);
    leftSensor.setMeasurementTimingBudget(50000);
  
    // Start continuous readings at a rate of one measurement every 50 ms (the
    // inter-measurement period). This period should be at least as long as the
    // timing budget.
    leftSensor.startContinuous(50); 
  }

  digitalWrite(4,LOW);//turn off left sensor
  digitalWrite(2,HIGH);//turn on right sensor
  
  rightSensor.setTimeout(500);
  if (!rightSensor.init()){
    functioningRight = false;
  } else {
    rightSensor.setAddress(2);
    rightSensor.setDistanceMode(VL53L1X::Medium);
    rightSensor.setMeasurementTimingBudget(50000);
  
    // Start continuous readings at a rate of one measurement every 50 ms (the
    // inter-measurement period). This period should be at least as long as the
    // timing budget.
    rightSensor.startContinuous(50); 
  }
  
}

void loop() {
  if(counter==0){
    Serial.write(0);
    counter=1;
  } else if(counter==1 && !leftSensor.timeoutOccurred()){
    if(functioningLeft){
      leftSensor.read();
      float floatInches = (float)(leftSensor.ranging_data.range_mm * 0.0393701);
      floatInches = floatInches * 2.0;
      int intInches = (int)(floatInches);
      Serial.write(intInches);
    } else {
      Serial.write(1);
    }
    counter=2;
  } else if(!rightSensor.timeoutOccurred()){
    if(functioningRight){
      rightSensor.read();
      float floatInches = (float)(rightSensor.ranging_data.range_mm * 0.0393701);
      floatInches = floatInches * 2.0;
      int intInches = (int)(floatInches);
      Serial.write(intInches);
    } else {
      Serial.write(1);
    }
    counter = 0;
  }
  
  
}
