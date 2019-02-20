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
    //Set followers and inverts for drive motors
    rMotorFront->SetInverted(true);
    rMotorBack->Follow(*rMotorFront, false);
    lMotorFront->SetInverted(false);
    lMotorBack->Follow(*lMotorFront, false);

    //Set current limit for drive motors
    rMotorFront->SetSmartCurrentLimit(driveMotorCurrentLimit);
    lMotorFront->SetSmartCurrentLimit(driveMotorCurrentLimit);
    rMotorBack->SetSmartCurrentLimit(driveMotorCurrentLimit);
    lMotorBack->SetSmartCurrentLimit(driveMotorCurrentLimit);
    
    //Config elevator motor
    elevatorMotor->SelectProfileSlot(0, 0);
    elevatorMotor->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 0);
    elevatorMotor->SetName("Elevator Motor");
    elevatorMotor->SetSelectedSensorPosition(0);
    elevatorMotor->Config_kP(0, pConstantElevator, 0);
    elevatorMotor->Config_kI(0, iConstantElevator, 0);
    elevatorMotor->Config_kD(0, dConstantElevator, 0);

    //Name the other talons
    cargoIntakeMotor->SetName("Cargo Intake");
    cargoLeftMotor->SetName("Cargo Left");
    cargoRightMotor->SetName("Cargo Right");
    cargoTopMotor->SetName("Cargo Top");

    SmartDashboard::PutBoolean("Rumble Driver Joystick", rumbleDriver);
    SmartDashboard::PutBoolean("Rumble Operator Joystick", rumbleOperator);
    SmartDashboard::PutBoolean("Single Controller?", singleController);
    SmartDashboard::PutBoolean("Operator in cargo mode?", operatorIsInverted);
}

void Robot::RobotPeriodic()
{
    SmartDashboard::PutNumber("Elevator Position", elevatorMotor->GetSelectedSensorPosition(0));
    singleController = SmartDashboard::GetBoolean("Single Controller?", singleController);
    rumbleDriver = SmartDashboard::GetBoolean("Rumble Driver Joystick", rumbleDriver);
    rumbleOperator = SmartDashboard::GetBoolean("Rumble Operator Joystick", rumbleOperator);
    operatorIsInverted = SmartDashboard::GetBoolean("Operator in cargo mode?", operatorIsInverted);

    if(rumbleDriver)
    {
        driverStick->SetRumble(GenericHID::RumbleType::kLeftRumble, 1);
        driverStick->SetRumble(GenericHID::RumbleType::kRightRumble, 1);
    }
    else
    {
        driverStick->SetRumble(GenericHID::RumbleType::kLeftRumble, 0);
        driverStick->SetRumble(GenericHID::RumbleType::kRightRumble, 0);
    }

    if(rumbleOperator)
    {
        operatorStick->SetRumble(GenericHID::RumbleType::kLeftRumble, 1);
        operatorStick->SetRumble(GenericHID::RumbleType::kRightRumble, 1);
    }
    else
    {
        operatorStick->SetRumble(GenericHID::RumbleType::kLeftRumble, 0);
        operatorStick->SetRumble(GenericHID::RumbleType::kRightRumble, 0);
    }
}

void Robot::TeleopInit()
{
    myRobot->ArcadeDrive(0, 0);
}

void Robot::TeleopPeriodic()
{
    //Invert button check
    if(driverStick->GetRawButton(7)) //Back button un-inverts controls
    {
        driverIsInverted = false;
    }
    else if(driverStick->GetRawButton(8)) // Start button inverts controls
    {
        driverIsInverted = true;
    }
    if(operatorStick->GetRawButton(7)) //Back button un-inverts controls
    {
        operatorIsInverted = false;
    }
    else if(driverStick->GetRawButton(8)) // Start button inverts controls
    {
        operatorIsInverted = true;
    }

    //Drive
    if(driverIsInverted)
    {
        myRobot->ArcadeDrive(-1.0 * driverStick->GetRawAxis(1), driverStick->GetRawAxis(4));
    }
    else
    {
        myRobot->ArcadeDrive(driverStick->GetRawAxis(1), -1.0 * driverStick->GetRawAxis(4));
    }

    //Operator Granular Elevator Control
    if(operatorStick->GetRawAxis(1) > 0.1 || operatorStick->GetRawAxis(1) < -0.1)
    {
        elevatorMotor->Set(operatorStick->GetRawAxis(1) * 0.3);
    }
    else if(elevatorMotor->GetControlMode() == ctre::phoenix::motorcontrol::ControlMode::PercentOutput) //Don't influence elevator motor if it's in position control mode
    {
        elevatorMotor->Set(0);
    }
    
    //Operator elevator levels
    if(operatorStick->GetRawButton(1)) //ground level (A button)
    {
        elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, 0);
    }
    if(operatorStick->GetRawButton(3)) //level 1 (X button)
    {
        if(operatorIsInverted)
        {
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, hatchRocket1);
        }
        else
        {
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, cargoRocket1);
        }
    }
    if(operatorStick->GetRawButton(2)) //level 2 (B button)
    {
        if(operatorIsInverted)
        {
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, hatchRocket2);
        }
        else
        {
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, cargoRocket2);
        }
    }
    if(operatorStick->GetRawButton(4)) //level 3 (Y button)
    {
        if(operatorIsInverted)
        {
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, hatchRocket3);
        }
        else
        {
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, cargoRocket3);
        }
    }
    if(operatorStick->GetRawButton(10)) //Cargo ship (push down on the secondary joystick)
    {
        if(operatorIsInverted)
        {
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, hatchRocket1); //hatchRocket1 = level for putting hatches on the cargo ship
        }
        else
        {
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, cargoShip);
        }
    }
}

void Robot::AutonomousInit()
{
    myRobot->initPID();
}


void Robot::AutonomousPeriodic()
{
    //Teleop?
}

void Robot::TestPeriodic()
{
    
}

void Robot::DisabledInit()
{
    myRobot->stopAutoThread();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
