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
    SmartDashboard::PutBoolean("Operator in cargo mode?", operatorInCargoMode);
}

void Robot::RobotPeriodic()
{
    SmartDashboard::PutNumber("Elevator Position", elevatorMotor->GetSelectedSensorPosition(0));
    singleController = SmartDashboard::GetBoolean("Single Controller?", singleController);
    rumbleDriver = SmartDashboard::GetBoolean("Rumble Driver Joystick", rumbleDriver);
    rumbleOperator = SmartDashboard::GetBoolean("Rumble Operator Joystick", rumbleOperator);
    operatorInCargoMode = SmartDashboard::GetBoolean("Operator in cargo mode?", operatorInCargoMode);

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
    if(singleController)
    {
        operatorStick = driverStick;
    }
}

void Robot::TeleopPeriodic()
{
    //Invert button check
    if(driverStick->GetRawButton(7) && !singleController) //Back button un-inverts controls
    {
        DriverStation::ReportError("Driver Mode: Uninverted");
        driverIsInverted = false;
    }
    else if(driverStick->GetRawButton(8) && !singleController) // Start button inverts controls
    {
        DriverStation::ReportError("Driver Mode: Inverted");
        driverIsInverted = true;
    }
    if(operatorStick->GetRawButton(7)) //Back button sets mode to cargo
    {
        DriverStation::ReportError("Operator Mode: Cargo");
        operatorInCargoMode = true;
    }
    else if(operatorStick->GetRawButton(8)) // Start button sets mode to hatch
    {
        DriverStation::ReportError("Operator Mode: Hatch");
        operatorInCargoMode = false;
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
    if(!singleController)
    {
        if(operatorStick->GetRawAxis(1) > 0.1 || operatorStick->GetRawAxis(1) < -0.1)
        {
            //THESE ASSUMPTIONS ARE PROBABLY INCORRECT
            elevatorMotor->Set(operatorStick->GetRawAxis(1) * 0.3);
        }
        else if(elevatorMotor->GetControlMode() == ctre::phoenix::motorcontrol::ControlMode::PercentOutput) //Don't influence elevator motor if it's in position control mode
        {
            elevatorMotor->Set(0);
        }   
    }
    
    //Operator elevator levels
    if(operatorStick->GetRawButtonPressed(1)) //ground level (A button)
    {
        DriverStation::ReportError("Elevator Set to Ground Level");
        elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, 0);
    }
    if(operatorStick->GetRawButtonPressed(3)) //level 1 (X button)
    {
        if(operatorInCargoMode)
        {
            DriverStation::ReportError("Elevator Set to Cargo Rocket 1");
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, cargoRocket1);
        }
        else
        {
            DriverStation::ReportError("Elevator Set to Hatch Rocket 1");
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, hatchRocket1);
        }
    }
    if(operatorStick->GetRawButtonPressed(2)) //level 2 (B button)
    {
        if(operatorInCargoMode)
        {
            DriverStation::ReportError("Elevator Set to Cargo Rocket 2");
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, cargoRocket2);
        }
        else
        {
            DriverStation::ReportError("Elevator Set to Hatch Rocket 2");
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, hatchRocket2);
        }
    }
    if(operatorStick->GetRawButtonPressed(4)) //level 3 (Y button)
    {
        if(operatorInCargoMode)
        {
            DriverStation::ReportError("Elevator Set to Cargo Rocket 3");
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, cargoRocket3);
        }
        else
        {
            DriverStation::ReportError("Elevator Set to Hatch Rocket 3");
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, hatchRocket3);
        }
    }
    if(operatorStick->GetRawButtonPressed(10)) //Cargo ship (push down on the secondary joystick)
    {
        if(operatorInCargoMode)
        {
            DriverStation::ReportError("Elevator Set to Cargo Ship");
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, cargoShip);
        }
        else
        {
            DriverStation::ReportError("Elevator Set to Hatch Ship");
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, hatchRocket1); //hatchRocket1 = level for putting hatches on the cargo ship
        }
    }

    //Trigger logic
    if(operatorInCargoMode)
    {
        hatchMechSolenoid->Set(false); //Idiot proofing hatch mech solenoid
        if(operatorStick->GetRawButtonPressed(6)) //intake pneumatics out (right bumper button)
        {
            DriverStation::ReportError("Intake Pneumatics Out");
            cargoMechLeftSolenoid->Set(true);
            cargoMechLeftSolenoid->Set(true);
        }
        if(operatorStick->GetRawButtonPressed(5)) //intake pneumatics in (left bumper button)
        {
            DriverStation::ReportError("Intake Pneumatics In");
            cargoMechLeftSolenoid->Set(false);
            cargoMechLeftSolenoid->Set(false);
        }

        if(operatorStick->GetRawAxis(2) > 0.5) //Intake wheels in (left trigger)
        {
            //THESE ASSUMPTIONS ARE PROBABLY INCORRECT
            cargoLeftMotor->Set(-1);
            cargoRightMotor->Set(-1);
            cargoIntakeMotor->Set(-1);
            outputtingCargo = false;
        }
        else if(operatorStick->GetRawAxis(3) > 0.5) //Intake wheels out (right trigger)
        {
            //THESE ASSUMPTIONS ARE PROBABLY INCORRECT
            cargoLeftMotor->Set(1);
            cargoRightMotor->Set(1);
            if(!outputtingCargo)
            {
                outputtingCargo = true;
                outputtingCargoStartTime = Timer().GetFPGATimestamp();
                DriverStation::ReportError("Intake Wheels Out");
            }
            else if(Timer().GetFPGATimestamp() - outputtingCargoStartTime > 0.5) //if it's been half a second
            {
                //THESE ASSUMPTIONS ARE PROBABLY INCORRECT
                cargoIntakeMotor->Set(1);
                DriverStation::ReportError("Intake Wheels Fire");
            }
        }
        else
        {
            outputtingCargo = false;
            cargoLeftMotor->Set(0);
            cargoRightMotor->Set(0);
            cargoIntakeMotor->Set(0);
        }
    }
    else
    {
        cargoMechLeftSolenoid->Set(false); //Idiot proofing cargo mech solenoid
        cargoMechRightSolenoid->Set(false); //Idiot proofing cargo mech solenoid
        if(operatorStick->GetRawAxis(2) > 0.5) //Servo down (left trigger)
        {
            hatchMechServo->SetAngle(servoDownAngle);
        }

        if(operatorStick->GetRawAxis(3) > 0.5) //Servo up (right trigger)
        {
            hatchMechServo->SetAngle(servoUpAngle);
            hatchMechSolenoid->Set(true);
        }
        else
        {
            hatchMechSolenoid->Set(false);
        }
    }
}

void Robot::AutonomousInit()
{
    TeleopInit();
}


void Robot::AutonomousPeriodic()
{
    //Teleop?
    TeleopPeriodic();
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
