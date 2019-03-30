#ifndef ULTRA_H
#define ULTRA_H
#include <frc/Ultrasonic.h>
using namespace frc;

class Ultra {
 public:
    Ultrasonic *leftSensor;
    Ultrasonic *rightSensor;
    double leftDist;
    double rightDist;
    double angle;
    double const diffBtwSensors = 12.0; //distance between the two sensors
    
    Ultra(int portNum1, int portNum2, int portNum3, int portNum4);
    
    double getLeftDist();
    double getRightDist();
    double getAngle();
};
#endif