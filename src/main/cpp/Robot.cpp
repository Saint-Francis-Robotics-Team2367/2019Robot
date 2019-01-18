/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Ultrasonic.h>
#include <frc/DigitalOutput.h>
#include <frc/DigitalInput.h>
#include <frc/DriverStation.h>
#include <string>
#include <frc/AnalogInput.h>
#include <math.h>
using namespace frc;

#define PI 3.14159265

//Creating objects
  Ultrasonic *ultraL, *ultraR;

void Robot::RobotInit(){

}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
 //HC-SR04
    ultraL = new Ultrasonic(2, 3);
	  ultraL->SetAutomaticMode(true);
    ultraR = new Ultrasonic(0, 1);
	  ultraR->SetAutomaticMode(true);
}

void Robot::AutonomousPeriodic() {
 //HC-SR04
    double diffBtwSensors = 12; // how far the sensors are from each other
    double rangeL = ultraL->GetRangeInches(); // left sensor distance to object
    double rangeR = ultraR->GetRangeInches(); // right sensor distance to object
    double diffBtwRange = abs(rangeL - rangeR); 
    double angle = atan(diffBtwSensors/diffBtwRange) * 180 / PI;
    DriverStation::ReportError("Angle: " + std::to_string(angle));

}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
