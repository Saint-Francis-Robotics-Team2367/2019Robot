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
#include <Ultra.h>
#include <frc/DigitalOutput.h>
#include <frc/DigitalInput.h>
#include <frc/DriverStation.h>
#include <string>
#include <frc/AnalogInput.h>

using namespace frc;

void Robot::RobotInit(){}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() 
{
    DriverStation::ReportError("Angle: " + std::to_string(ultra.getAngle()));
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
