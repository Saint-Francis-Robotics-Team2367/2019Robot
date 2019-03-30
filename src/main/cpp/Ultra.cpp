#include<Ultra.h>
#include<frc/Ultrasonic.h>
#include<math.h>
#define PI 3.14159265
using namespace frc;

Ultra::Ultra(int portNum1, int portNum2, int portNum3, int portNum4)
{
    // create the sensor objects
    leftSensor = new Ultrasonic(portNum1, portNum2);
    rightSensor = new Ultrasonic(portNum3, portNum4);

    // turns on automatic mode
    leftSensor->SetAutomaticMode(true);
    rightSensor->SetAutomaticMode(true);

    // initialize distances and angle
    leftDist = 0.0;
    rightDist = 0.0;
    angle = 0.0;
}

double Ultra::getLeftDist()
// return left sensor's range
{
    leftDist = leftSensor->GetRangeInches(); // left sensor distance to object
    return leftDist;
}

double Ultra::getRightDist()
// return right sensor's range
{
    rightDist = rightSensor->GetRangeInches(); // right sensor distance to object
    return rightDist;
}

double Ultra::getAngle(){
// return the angle between robot and wall
    double diffBtwRange = abs(getLeftDist() - getRightDist()); 
    angle = atan(diffBtwSensors/diffBtwRange) * 180 / PI;
    return angle;
}