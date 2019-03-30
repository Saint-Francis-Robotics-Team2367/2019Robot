#include<Ultra.h>
#include<frc/Ultrasonic.h>
#include<math.h>
#define PI 3.14159265
using namespace frc;

Ultra::Ultra(int portNum1, int portNum2, int portNum3, int portNum4){

    leftSensor = new Ultrasonic(portNum1, portNum2);
    rightSensor = new Ultrasonic(portNum3, portNum4);

    leftSensor->SetAutomaticMode(true);
    rightSensor->SetAutomaticMode(true);

    leftDist = 0.0;
    rightDist = 0.0;
    angle = 0.0;
}
double Ultra::getLeftDist(){
    leftDist = leftSensor->GetRangeInches(); // left sensor distance to object
    return leftDist;
}
double Ultra::getRightDist(){
    rightDist = rightSensor->GetRangeInches(); // right sensor distance to object
    return rightDist;
}
double Ultra::getAngle(){
    double diffBtwRange = abs(leftDist - rightDist); 
    angle = atan(diffBtwSensors/diffBtwRange) * 180 / PI;
    return angle;
}