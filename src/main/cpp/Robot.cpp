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
    lMotorFront->ConfigSelectedFeedbackSensor(qE, 0, checkTimeout);
    rMotorFront->ConfigSelectedFeedbackSensor(qE, 0, checkTimeout);
    sparkRight->SetInverted(true);
    sparkRightSlave->Follow(*sparkRight, false);


    //Use when controller forward/reverse output doesn't correlate to appropriate forward/reverse reading of sensor
    lMotorFront->SetSensorPhase(false);
    lMotorBack->SetSensorPhase(false);

    //set back motors to follower mode
    rMotorBack->Set(ctre::phoenix::motorcontrol::ControlMode::Follower, rMotorFrontNum);
    lMotorBack->Set(ctre::phoenix::motorcontrol::ControlMode::Follower, lMotorFrontNum);

    //set the names of the talons
    lMotorFront->SetName("Left Front");
    rMotorFront->SetName("Right Front");
    lMotorBack->SetName("Left Back");
    rMotorBack->SetName("Right Back");

     //Set drive motor max voltage to 30 amps and current
    lMotorFront->ConfigContinuousCurrentLimit(maxDriveMotorCurrent - 5, checkTimeout);
    rMotorFront->ConfigContinuousCurrentLimit(maxDriveMotorCurrent - 5, checkTimeout);
    lMotorBack->ConfigContinuousCurrentLimit(maxDriveMotorCurrent - 5, checkTimeout);
    rMotorBack->ConfigContinuousCurrentLimit(maxDriveMotorCurrent - 5, checkTimeout);

    lMotorFront->ConfigPeakCurrentLimit(maxDriveMotorCurrent, checkTimeout);
    rMotorFront->ConfigPeakCurrentLimit(maxDriveMotorCurrent, checkTimeout);
    lMotorBack->ConfigPeakCurrentLimit(maxDriveMotorCurrent, checkTimeout);
    rMotorBack->ConfigPeakCurrentLimit(maxDriveMotorCurrent, checkTimeout);

    lMotorFront->ConfigPeakCurrentDuration(0, checkTimeout);
    rMotorFront->ConfigPeakCurrentDuration(0, checkTimeout);
    lMotorBack->ConfigPeakCurrentDuration(0, checkTimeout);
    rMotorBack->ConfigPeakCurrentDuration(0, checkTimeout);

    lMotorFront->EnableCurrentLimit(true);
    rMotorFront->EnableCurrentLimit(true);
    lMotorBack->EnableCurrentLimit(true);
    rMotorBack->EnableCurrentLimit(true);

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
    sparks->initPID();
    sparks->PIDDrive(10, 10);
}


void Robot::AutonomousPeriodic()
{
    DriverStation::ReportError(std::to_string(sparkLeft->GetEncoder().GetPosition()));
}

void Robot::TeleopInit()
{
    DriverStation::ReportError("TeleopInit Started");
    //Set encoder positions to 0
    ConfigPIDS();
    sparks->ArcadeDrive(0.0, 0.0);
    DriverStation::ReportError("TeleopInit Completed");

    //list testing block in shuffleboard.
    SmartDashboard::PutNumber("maxVel", 55.0);
    SmartDashboard::PutNumber("auto Timeout", 4.0);
    SmartDashboard::PutNumber("maxAccl", 10000);
    DriverStation::ReportError("TestInit Completed");
}

void Robot::TeleopPeriodic()
{
    if(stick->GetRawAxis(4)>=0){
        turning = stick->GetRawAxis(4) * stick->GetRawAxis(4);
    }else{
        turning = -1.0 *( stick->GetRawAxis(4) * stick->GetRawAxis(4));
    }
    sparks->ArcadeDrive(stick->GetRawAxis(1), -1.0 *turning);
    
}

void Robot::TestPeriodic()
{

}

void Robot::ConfigPIDS()
{
    DriverStation::ReportError("PID Config Started");
    
    rMotorBack->SetNeutralMode(Brake);
    lMotorBack->SetNeutralMode(Brake);

    rMotorFront->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);
    rMotorBack->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);
    lMotorFront->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);
    lMotorBack->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);

    lMotorFront->Config_kP(0, pConstantDrive, checkTimeout);
    lMotorFront->Config_kI(0, iConstantDrive, checkTimeout);
    lMotorFront->Config_kD(0, dConstantDrive, checkTimeout);
    lMotorFront->Config_kF(0, fConstantDrive, checkTimeout);

    lMotorBack->Config_kP(0, pConstantDrive, checkTimeout);
    lMotorBack->Config_kI(0, iConstantDrive, checkTimeout);
    lMotorBack->Config_kD(0, dConstantDrive, checkTimeout);
    lMotorBack->Config_kF(0, fConstantDrive, checkTimeout);

    rMotorFront->Config_kP(0, pConstantDrive, checkTimeout);
    rMotorFront->Config_kI(0, iConstantDrive, checkTimeout);
    rMotorFront->Config_kD(0, dConstantDrive, checkTimeout);
    rMotorFront->Config_kF(0, fConstantDrive, checkTimeout);

    rMotorBack->Config_kP(0, pConstantDrive, checkTimeout);
    rMotorBack->Config_kI(0, iConstantDrive, checkTimeout);
    rMotorBack->Config_kD(0, dConstantDrive, checkTimeout);
    rMotorBack->Config_kF(0, fConstantDrive, checkTimeout);

    DriverStation::ReportError("PID Config Completed");
}

void Robot::DisabledInit()
{
    myRobot->stopAutoThread();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
