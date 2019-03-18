/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

enum JoystickButtons {A_BUTTON = 1, B_BUTTON = 2, X_BUTTON = 3, Y_BUTTON = 4, LEFT_BUMPER = 5, RIGHT_BUMPER = 6, BACK_BUTTON = 7, START_BUTTON = 8, LEFT_JOYSTICK_BUTTON = 9, RIGHT_JOYSTICK_BUTTON = 10};
enum JoystickAxes {L_X_AXIS = 0, L_Y_AXIS = 1, L_TRIGGER = 2, R_TRIGGER = 3, R_X_AXIS = 4, R_Y_AXIS = 5};

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
    elevatorMotor->EnableCurrentLimit(true);
    elevatorMotor->ConfigContinuousCurrentLimit(20);
    elevatorMotor->ConfigPeakCurrentDuration(30);
    elevatorMotor->ConfigPeakCurrentLimit(5);

    //Name the other talons
    cargoIntakeMotor->SetName("Cargo Intake");
    cargoLeftMotor->SetName("Cargo Left");
    cargoRightMotor->SetName("Cargo Right");
    cargoTopMotor->SetName("Cargo Top");

    SmartDashboard::PutBoolean("Rumble Driver Joystick", rumbleDriver);
    SmartDashboard::PutBoolean("Rumble Operator Joystick", rumbleOperator);
    SmartDashboard::PutBoolean("Single Controller?", singleController);
    SmartDashboard::PutBoolean("Operator in cargo mode?", operatorInCargoMode);

    //Test stuff
    sender->addNumber(&servoUpAngle, "Servo up angle (TICKS NOT DEGREES)");
    sender->addNumber(&servoDownAngle, "Servo down angle (TICKS NOT DEGREES)");
    sender->addNumber(&setpoint, "Setpoint");
}

void Robot::RobotPeriodic()
{
    SmartDashboard::PutNumber("Elevator Position", elevatorMotor->GetSelectedSensorPosition(0));
    singleController = SmartDashboard::GetBoolean("Single Controller?", singleController);
    rumbleDriver = SmartDashboard::GetBoolean("Rumble Driver Joystick", rumbleDriver);
    rumbleOperator = SmartDashboard::GetBoolean("Rumble Operator Joystick", rumbleOperator);
    operatorInCargoMode = SmartDashboard::GetBoolean("Operator in cargo mode?", operatorInCargoMode);
    
    sender->getNumbers();

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
    if(!singleController)
    {
        //Driver has drivetrain control
        myRobot->ArcadeDrive(driverStick->GetRawAxis(JoystickAxes::L_Y_AXIS), -1.0 * driverStick->GetRawAxis(JoystickAxes::R_X_AXIS));

        //Driver Granular Elevator Control
        if(driverStick->GetRawAxis(JoystickAxes::L_TRIGGER) > 0.1)
        {
            elevatorMotor->Set(-1.0 * driverStick->GetRawAxis(JoystickAxes::L_TRIGGER) * elevatorGranularControlMultiplier);
        }
        else if(driverStick->GetRawAxis(JoystickAxes::R_TRIGGER) > 0.1)
        {
            elevatorMotor->Set(driverStick->GetRawAxis(JoystickAxes::R_TRIGGER) * elevatorGranularControlMultiplier);
        }
        else if(elevatorMotor->GetControlMode() == ctre::phoenix::motorcontrol::ControlMode::PercentOutput) //Don't influence elevator motor if it's in position control mode
        {
            elevatorMotor->Set(0);
        }

        //Driver Cargo Intake Controls
        if(driverStick->GetRawButton(JoystickButtons::LEFT_BUMPER)) //Intake wheels in (LEFT BUMPER)
        {
            //THESE ASSUMPTIONS ARE PROBABLY INCORRECT
            cargoLeftMotor->Set(-1);
            cargoRightMotor->Set(-1);
            cargoIntakeMotor->Set(-1);
            cargoTopMotor->Set(-1);
            outputtingCargo = false;
        }
        else if(driverStick->GetRawAxis(JoystickButtons::RIGHT_BUMPER)) //Intake wheels out (RIGHT BUMPER)
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
                cargoTopMotor->Set(1);
                DriverStation::ReportError("Intake Wheels Fire");
            }
        }
        else
        {
            outputtingCargo = false;
            cargoLeftMotor->Set(0);
            cargoRightMotor->Set(0);
            cargoTopMotor->Set(0);
            cargoIntakeMotor->Set(0);
        }

        //Driver Cargo Pneumatic Controls
        if(driverStick->GetRawButton(JoystickButtons::A_BUTTON)) //Retract cargo mech pneumatics (A BUTTON)
        {
            cargoMechLeftSolenoid->Set(false);
            cargoMechRightSolenoid->Set(false);
        }
        if(driverStick->GetRawButton(JoystickButtons::X_BUTTON)) //Extend cargo mech pneumatics (X BUTTON)
        {
            cargoMechLeftSolenoid->Set(true);
            cargoMechRightSolenoid->Set(true);
        }


        //Operator Elevator Control
        if(operatorStick->GetRawButtonPressed(JoystickButtons::START_BUTTON) || operatorStick->GetPOV() == 180 || operatorStick->GetPOV() == 270) //ground level (START button, DPAD DOWN, DPAD LEFT)
        {
            DriverStation::ReportError("Elevator Set to Ground Level (or hatch level 1)");
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, 10);
        }
        if(operatorStick->GetRawButtonPressed(JoystickButtons::A_BUTTON)) //Hatch level cargo ship (A button)
        {
            DriverStation::ReportError("Elevator Set to Ball Level CargoShip");
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, cargoShip);
        }
        if(operatorStick->GetRawButtonPressed(JoystickButtons::X_BUTTON)) //Hatch rocket level 1 (X button)
        {
            DriverStation::ReportError("Elevator Set to Ball Level 1");
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, cargoRocket1);
        }
        if(operatorStick->GetRawButtonPressed(JoystickButtons::B_BUTTON)) //Hatch rocket level 2 (B button)
        {
            DriverStation::ReportError("Elevator Set to Ball Level 2");
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, cargoRocket2);
        }
        if(operatorStick->GetRawButtonPressed(JoystickButtons::Y_BUTTON)) //Hatch rocket level 3 (Y button)
        {
            DriverStation::ReportError("Elevator Set to Ball Level 3");
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, cargoRocket3);
        }
        if(operatorStick->GetPOV() == 90) //Hatch rocket level 2 (DPAD RIGHT)
        {
            DriverStation::ReportError("Elevator Set to Hatch Level 2");
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, hatchRocket2);
        }
        if(operatorStick->GetPOV() == 0) //Hatch rocket level 3 (DPAD UP)
        {
            DriverStation::ReportError("Elevator Set to Hatch Level 3");
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, hatchRocket3);
        }
    }
    else //Single controller mode is old joystick mappings
    {
        //Driver has drivetrain control
        myRobot->ArcadeDrive(driverStick->GetRawAxis(JoystickAxes::L_Y_AXIS), -1.0 * driverStick->GetRawAxis(JoystickAxes::R_X_AXIS));

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
            if(operatorStick->GetRawButtonPressed(6)) //intake pneumatics out (right trigger button)
            {
                DriverStation::ReportError("Intake Pneumatics Out");
                cargoMechLeftSolenoid->Set(true);
                cargoMechLeftSolenoid->Set(true);
            }
            if(operatorStick->GetRawButtonPressed(5)) //intake pneumatics out (right trigger button)
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
}

void Robot::AutonomousInit()
{
    
}


void Robot::AutonomousPeriodic()
{
    myRobot->ArcadeDrive(driverStick->GetRawAxis(JoystickAxes::L_X_AXIS), -1.0 * driverStick->GetRawAxis(JoystickAxes::R_X_AXIS));
    if(driverStick->GetRawButton(JoystickButtons::RIGHT_BUMPER))
    {
        elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
    }
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
