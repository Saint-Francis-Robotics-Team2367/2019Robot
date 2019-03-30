#include <iostream>

#include <math.h>

#include "MotionProfile.h"

#include <frc/Timer.h>

motionProfiler::motionProfiler(double _accel, double _maxSpeed, double _finalPos) {
    accel = _accel;
    maxSpeed = _maxSpeed;
    finalPos = _finalPos;

    /*
    The time spent accelerating to linear (and the time spent decelerating)
    maxSpeed
    --------
      accel
    */
    timeToLinear = maxSpeed / accel;
    /*
    The time spent in linear at max speed until decelerating
    (finalPos - (accel*timeSpentAccel^2))
    -------------------------------------
                  maxSpeed
    */

    timeToDecel = ((finalPos - (accel*(pow(timeToLinear, 2.0))))/maxSpeed); // shift it sideways with the acceleration portion
     if(timeToDecel <= 0) { // is there even a linear phase?
        timeToDecel = 0;
        timeToLinear = sqrt(finalPos/accel);
        maxSpeed = accel*timeToLinear;
    }
    /*
    The time we should be at the end is defined as the two times the time spent accel/decel (assuming symmetric acceleration and deceleration)
    plus the time we spent in the linear phase
    */
    timeToStop = 2*timeToLinear + timeToDecel;
}
double motionProfiler::getFinalPos() {
    return finalPos;
}
double motionProfiler::getValue(frc::Timer* timer) {
    if(timer->Get() <= timeToLinear) {
        
        //If we are still accelerating, return 1/2at^2
        //return 0.5*accel*(pow(timer->Get(), 2.0));
        return accel*timer->Get()
        ;
    } else if(timer->Get() > timeToLinear && timer->Get() <= (timeToDecel + timeToLinear)) {

        // If we are in linear, return the line Vmax*t + distance when we stopped accelerating	
        //return (maxSpeed*(timer->Get() - timeToLinear)) + 0.5*accel*pow(timeToLinear, 2.0);
        return maxSpeed;
    } else if(timer->Get() > (timeToDecel + timeToLinear) && timer->Get() < timeToStop) {
        
		// If we should be decelerating, return the inverse of the accel curve plus the distance we have already travelled
        
		
		/* distance accelerated + distance spent at linear + distance that causes deceleration
		
		Implementation of ViT + 0.5at^2

		Assuming Accel is symmetric, the decel will be the negative accel
		The initial velocity is the max speed, since we were just in linear
		The time is time - (timeToLinear + timeToDecel) because we're treating the equation as though it is restarted at each phase, so the time is zeroed every time
		*/
		//return (0.5*accel*pow(timeToLinear, 2.0)) + (maxSpeed*timeToDecel) + (maxSpeed*(timer->Get() - (timeToLinear + timeToDecel))) - 0.5*accel*pow(timer->Get() - (timeToLinear + timeToDecel), 2.0);
        return maxSpeed - (accel*(timer->Get() - (timeToLinear + timeToDecel)));
    }

	return finalPos;

}