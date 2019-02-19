/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>


void Robot::RobotInit() 
{
    //set the encoders to quadrature
    ctre::phoenix::motorcontrol::FeedbackDevice qE = QuadEncoder;
    rMotorFront->SetInverted(true);
    rMotorBack->Follow(*rMotorFront, false);
    lMotorFront->SetInverted(false);
    lMotorBack->Follow(*lMotorFront, false);

    //Shuffle board 
    SmartDashboard::PutNumber("Rumble Multiplier", rumbleMultiplier);
    SmartDashboard::PutNumber("Rumble Deadzone", rumbleDeadzone);
    //SmartDashboard::PutNumber("maxVel", 55.0);
    SmartDashboard::PutNumber("auto Timeout", 4.0);
    SmartDashboard::PutNumber("maxAccl", 10000);

    //config acceleraometer
    accelerometer.SetRange(frc::Accelerometer::kRange_8G);
}

void Robot::RobotPeriodic()
{

}

void Robot::AutonomousInit()
{
    myRobot->initPID();
    lMotorFront->GetPIDController().SetReference(-10, rev::ControlType::kPosition, 0, 0);
    rMotorFront->GetPIDController().SetReference(10, rev::ControlType::kPosition, 0, 0);
}


void Robot::AutonomousPeriodic()
{
    DriverStation::ReportError(std::to_string(lMotorFront->GetEncoder().GetPosition()));
}

void Robot::TeleopInit()
{
    DriverStation::ReportError("TeleopInit Started");
    //Set encoder positions to 0
    ConfigPIDS();
    myRobot->ArcadeDrive(0.0, 0.0);
    DriverStation::ReportError("TeleopInit Completed");

    //list testing block in shuffleboard.
    SmartDashboard::PutNumber("maxVel", 55.0);
    SmartDashboard::PutNumber("auto Timeout", 4.0);
    SmartDashboard::PutNumber("maxAccl", 10000);
    DriverStation::ReportError("TestInit Completed");
}

void Robot::TeleopPeriodic()
{
    myRobot->ArcadeDrive(stick->GetRawAxis(1), -1.0 * stick->GetRawAxis(4));
    
    if(stick->GetRawAxis(3))
    {
        motorA->Set(stick->GetRawAxis(3));
    }
    else if(stick->GetRawAxis(2))
    {
        motorA->Set(-1.0 * stick->GetRawAxis(2));
    }
    else
    {
        motorA->Set(0);
    }

    if(stick->GetRawButton(4))
    {
        motorB->Set(1);
    }
    else if(stick->GetRawButton(2))
    {
        motorB->Set(-1);
    }
    else
    {
        motorB->Set(0);
    }

    if(stick->GetRawButton(3))
    {
        motorC->Set(1);
    }
    else if(stick->GetRawButton(1))
    {
        motorC->Set(-1);
    }
    else
    {
        motorC->Set(0);
    }
}

void Robot::TestPeriodic()
{
    
}

void Robot::ConfigPIDS()
{
    DriverStation::ReportError("PID Config Started");
    
    //todo

    DriverStation::ReportError("PID Config Completed");
}

void Robot::DisabledInit()
{
    myRobot->stopAutoThread();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
