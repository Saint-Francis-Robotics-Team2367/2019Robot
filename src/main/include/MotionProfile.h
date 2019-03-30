#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <frc/Timer.h>
class motionProfiler {
    private:
    double accel;
    double maxSpeed;
    double finalPos;

    double timeToLinear; // time it takes to accelerate up to max speed
    double timeToDecel; // time spent at max speed until decel
    double timeToStop;
    
    public:
    motionProfiler(double _accel, double _maxSpeed, double _finalPos);

    double getFinalPos();
    
    double getValue(frc::Timer* timer);

    
};
#endif