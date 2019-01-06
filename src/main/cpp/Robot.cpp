/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
    ctre::phoenix::motorcontrol::FeedbackDevice qE = QuadEncoder;
    _lMotorFront->ConfigSelectedFeedbackSensor(qE, 0, checkTimeout);
    _rMotorFront->ConfigSelectedFeedbackSensor(qE, 0, checkTimeout);

    _lMotorFront->SetSensorPhase(false);
    _lMotorBack->SetSensorPhase(false);
    _rMotorBack->Set(ctre::phoenix::motorcontrol::ControlMode::Follower, rMotorFrontNum);
    _lMotorBack->Set(ctre::phoenix::motorcontrol::ControlMode::Follower, lMotorFrontNum);

}

void Robot::RobotPeriodic() 
{

}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() 
{

}

void Robot::TeleopPeriodic() 
{

}

void Robot::TestPeriodic() 
{
  
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
