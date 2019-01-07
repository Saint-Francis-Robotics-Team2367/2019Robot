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
    //set the encoders to quadrature
    ctre::phoenix::motorcontrol::FeedbackDevice qE = QuadEncoder;
    _lMotorFront->ConfigSelectedFeedbackSensor(qE, 0, checkTimeout);
    _rMotorFront->ConfigSelectedFeedbackSensor(qE, 0, checkTimeout);

    _lMotorFront->SetSensorPhase(false);
    _lMotorBack->SetSensorPhase(false);

    //set back motors to follower mode
    _rMotorBack->Set(ctre::phoenix::motorcontrol::ControlMode::Follower, rMotorFrontNum);
    _lMotorBack->Set(ctre::phoenix::motorcontrol::ControlMode::Follower, lMotorFrontNum);

    //set the names of the talons
    _lMotorFront->SetName("Left Front");
    _rMotorFront->SetName("Right Front");
    _lMotorBack->SetName("Left Back");
    _rMotorBack->SetName("Right Back");

     //Set drive motor max voltage to 30 amps

    _lMotorFront->ConfigContinuousCurrentLimit(maxDriveMotorCurrent - 5, checkTimeout);
    _rMotorFront->ConfigContinuousCurrentLimit(maxDriveMotorCurrent - 5, checkTimeout);
    _lMotorBack->ConfigContinuousCurrentLimit(maxDriveMotorCurrent - 5, checkTimeout);
    _rMotorBack->ConfigContinuousCurrentLimit(maxDriveMotorCurrent - 5, checkTimeout);

    _lMotorFront->ConfigPeakCurrentLimit(maxDriveMotorCurrent, checkTimeout);
    _rMotorFront->ConfigPeakCurrentLimit(maxDriveMotorCurrent, checkTimeout);
    _lMotorBack->ConfigPeakCurrentLimit(maxDriveMotorCurrent, checkTimeout);
    _rMotorBack->ConfigPeakCurrentLimit(maxDriveMotorCurrent, checkTimeout);

    _lMotorFront->ConfigPeakCurrentDuration(0, checkTimeout);
    _rMotorFront->ConfigPeakCurrentDuration(0, checkTimeout);
    _lMotorBack->ConfigPeakCurrentDuration(0, checkTimeout);
    _rMotorBack->ConfigPeakCurrentDuration(0, checkTimeout);

    _lMotorFront->EnableCurrentLimit(true);
    _rMotorFront->EnableCurrentLimit(true);
    _lMotorBack->EnableCurrentLimit(true);
    _rMotorBack->EnableCurrentLimit(true);

    //Shuffle board 
    SmartDashboard::PutNumber("Rumble Multiplier", rumbleMultiplier);
    SmartDashboard::PutNumber("Rumble Deadzone", rumbleDeadzone);
    SmartDashboard::PutNumber("maxVel", 55.0);
    SmartDashboard::PutNumber("auto Timeout", 4.0);
    SmartDashboard::PutNumber("maxAccl", 10000);

    //config acceleraometer
    accelerometer.SetRange(frc::Accelerometer::kRange_8G);
}

void Robot::RobotPeriodic(){}
void Robot::AutonomousInit(){}
void Robot::AutonomousPeriodic(){}

void Robot::TeleopInit(){
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

void Robot::TeleopPeriodic(){
    myRobot->ArcadeDrive(scale * stick->GetRawAxis(1), -(stick->GetRawAxis(4) > 0 ? 1 : -1) * stick->GetRawAxis(4) * stick->GetRawAxis(4));

    myRobot->setAccel(SmartDashboard::GetNumber("maxAccl", 8000));
    SmartDashboard::PutNumber("Left Encoder", _lMotorFront->GetSelectedSensorPosition(0));
    SmartDashboard::PutNumber("Right Encoder", _rMotorFront->GetSelectedSensorPosition(0));

    //set the rumble
    double acceleration = std::pow(accelerometer.GetX() * accelerometer.GetX() + accelerometer.GetY() * accelerometer.GetY(), 0.5);
    rumbleMultiplier = SmartDashboard::GetNumber("Rumble Multiplier", rumbleMultiplier);
    rumbleDeadzone = SmartDashboard::GetNumber("Rumble Deadzone", rumbleDeadzone);
    if(acceleration < rumbleDeadzone){
        acceleration=0;
    }
    stick->SetRumble(GenericHID::RumbleType::kLeftRumble, acceleration * rumbleMultiplier);
    stick->SetRumble(GenericHID::RumbleType::kRightRumble, acceleration * rumbleMultiplier);
}

void Robot::TestPeriodic(){}

void Robot::ConfigPIDS(){
    DriverStation::ReportError("PID Config Started");
    
    _rMotorBack->SetNeutralMode(Brake);
    _lMotorBack->SetNeutralMode(Brake);

    _rMotorFront->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);
    _rMotorBack->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);
    _lMotorFront->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);
    _lMotorBack->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);

    _lMotorFront->Config_kP(0, pConstantDrive, checkTimeout);
    _lMotorFront->Config_kI(0, iConstantDrive, checkTimeout);
    _lMotorFront->Config_kD(0, dConstantDrive, checkTimeout);
    _lMotorFront->Config_kF(0, fConstantDrive, checkTimeout);

    _lMotorBack->Config_kP(0, pConstantDrive, checkTimeout);
    _lMotorBack->Config_kI(0, iConstantDrive, checkTimeout);
    _lMotorBack->Config_kD(0, dConstantDrive, checkTimeout);
    _lMotorBack->Config_kF(0, fConstantDrive, checkTimeout);

    _rMotorFront->Config_kP(0, pConstantDrive, checkTimeout);
    _rMotorFront->Config_kI(0, iConstantDrive, checkTimeout);
    _rMotorFront->Config_kD(0, dConstantDrive, checkTimeout);
    _rMotorFront->Config_kF(0, fConstantDrive, checkTimeout);

    _rMotorBack->Config_kP(0, pConstantDrive, checkTimeout);
    _rMotorBack->Config_kI(0, iConstantDrive, checkTimeout);
    _rMotorBack->Config_kD(0, dConstantDrive, checkTimeout);
    _rMotorBack->Config_kF(0, fConstantDrive, checkTimeout);

    DriverStation::ReportError("PID Config Completed");
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
