/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may de modified and shared by FRC teams. The code   */
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
    elevatorMotor->SetSensorPhase(true);
    elevatorMotor->SetInverted(true);
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
    elevatorMotor->SetSelectedSensorPosition(0);
    elevatorMotor->Config_kP(0, pConstantElevator, 0);
    elevatorMotor->Config_kI(0, iConstantElevator, 0);
    elevatorMotor->Config_kD(0, dConstantElevator, 0);
    elevatorMotor->EnableCurrentLimit(true);
    elevatorMotor->ConfigContinuousCurrentLimit(elevatorContinuousMotorCurrentLimit);
    elevatorMotor->ConfigPeakCurrentDuration(elevatorPeakMotorCurrentLimitDuration);
    elevatorMotor->ConfigPeakCurrentLimit(elevatorPeakMotorCurrentLimit);

    //Name the other talons
    cargoIntakeMotor->SetName("Cargo Intake");
    cargoLeftMotor->SetName("Cargo Left");
    cargoRightMotor->SetName("Cargo Right");
    cargoTopMotor->SetName("Cargo Top");

    SmartDashboard::PutBoolean("Rumble Driver Joystick", rumbleDriver);
    SmartDashboard::PutBoolean("Rumble Operator Joystick", rumbleOperator);

    //Test stuff
    sender->addNumber(&myRobot->m_lowSpeedControlMultiplier, "Low Speed Control Multiplier");
    sender->addNumber(&myRobot->m_highSpeedControlMultiplier, "High Speed Control Multiplier");
    sender->addNumber(&myRobot->m_deadband, "Deadzone");
    sender->addNumber(&myRobot->m_thresholdPercentage, "Threshold Percentage");

    sender->putNumbers();
}

void Robot::RobotPeriodic()
{
    rumbleDriver = SmartDashboard::GetBoolean("Rumble Driver Joystick", rumbleDriver);
    rumbleOperator = SmartDashboard::GetBoolean("Rumble Operator Joystick", rumbleOperator); 
    
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
    cargoMechLeftSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    cargoMechRightSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    hatchMechBottomServo->SetAngle(bottomServoUpSetpoint);
    hatchMechSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    lifterFrontSolenoid->Set(false);
    lifterBackSolenoid->Set(false);
}

void Robot::TeleopPeriodic()
{ 
    //Driver has drivetrain control
        myRobot->ModifiedAcadeDrive(driverStick->GetRawAxis(JoystickAxes::L_Y_AXIS), -1.0 * driverStick->GetRawAxis(JoystickAxes::R_X_AXIS));

    //Driver Granular Elevator Control
    if(driverStick->GetRawAxis(JoystickAxes::L_TRIGGER) > 0.05)
    {
        setpoint += 320 * driverStick->GetRawAxis(JoystickAxes::L_TRIGGER);
        if(setpoint > 400 && !elevatorFlag) //If elevator at bottom stop, then coast mode, set sensor to zero and trigger flag
        {
            elevatorMotor->SetSelectedSensorPosition(0, 0, 0);
            elevatorMotor->SetNeutralMode(NeutralMode::Coast);
            elevatorFlag = true;
        }
        else if(!elevatorFlag)
        {
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
        }
    }
    if(driverStick->GetRawAxis(JoystickAxes::R_TRIGGER) > 0.05)
    {
        setpoint -= 320 * driverStick->GetRawAxis(JoystickAxes::R_TRIGGER);
        if(setpoint < -40000) //If elevator close to top stop, don't break the god damn elevator
        {
            setpoint = -40000;
        }
        if(elevatorFlag && setpoint < -400) //If elevator flag triggered and setpoint > -420, set elevator to brake mode and untrigger flag
        {
            elevatorFlag = false;
            elevatorMotor->SetNeutralMode(NeutralMode::Brake);
        }
        if(!elevatorFlag)
        {
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
        }
    }
    if(!elevatorFlag)
    {
        elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
    }
    if(driverStick->GetRawButton(JoystickButtons::START_BUTTON) || operatorStick->GetRawButton(JoystickButtons::START_BUTTON))
    {
        if(elevatorMotor->GetSelectedSensorPosition() < cargoRocket1)
        {
            setpoint =  (int)(cargoRocket1/2);
            DriverStation::ReportError(std::to_string(setpoint));
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
            elevatorFlag = false;
            elevatorCargoLevel = false;
        }
        else
        {
            elevatorFlag = true;
            elevatorCargoLevel = false;
            setpoint = 0;
            //DriverStation::ReportError("In Coast Mode");
            elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
            elevatorMotor->SetNeutralMode(NeutralMode::Coast);
        }
        
    }

    //Driver Cargo Intake Controls
    if(driverStick->GetRawButton(JoystickButtons::LEFT_BUMPER)) //Intake wheels in (LEFT BUMPER)
    {
        cargoLeftMotor->Set(1);
        cargoRightMotor->Set(1);
        cargoIntakeMotor->Set(1);
        cargoTopMotor->Set(1);
        outputtingCargo = false;
    }
    else if(driverStick->GetRawButton(JoystickButtons::RIGHT_BUMPER)) //Intake wheels out (RIGHT BUMPER)
    {
        if(elevatorCargoLevel)
        {
            cargoLeftMotor->Set(-0.4);
            cargoRightMotor->Set(-0.4);
            cargoTopMotor->Set(-0.4);
            cargoIntakeMotor->Set(-0.4);
        }
        else
        {
            cargoLeftMotor->Set(-0.8);
            cargoRightMotor->Set(-0.8);
            cargoTopMotor->Set(-0.8);
            cargoIntakeMotor->Set(-0.8);
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
        cargoMechLeftSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
        cargoMechRightSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    }
    if(driverStick->GetRawButton(JoystickButtons::X_BUTTON)) //Extend cargo mech pneumatics (X BUTTON)
    {
        cargoMechLeftSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
        cargoMechRightSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
    }

    //Driver Hatch Mech Controls
    if(driverStick->GetRawButtonPressed(JoystickButtons::B_BUTTON)) //Iterates down through the stages
    {
        if(hatchMechState == 0) hatchMechState = totalHatchStages;
        else hatchMechState = hatchMechState - 1;
        hatchMechStateSwitched = true;
        DriverStation::ReportError("Hatch mech stage: " + std::to_string(hatchMechState));
    }
    if(driverStick->GetRawButtonPressed(JoystickButtons::Y_BUTTON)) //Iterates up through the stages
    {
        if(hatchMechState == totalHatchStages) hatchMechState = 0;
        else hatchMechState = hatchMechState + 1;
        hatchMechStateSwitched = true;
        DriverStation::ReportError("Hatch mech stage: " + std::to_string(hatchMechState));
    }
    if(driverStick->GetRawButton(JoystickButtons::BACK_BUTTON) || operatorStick->GetRawButton(JoystickButtons::BACK_BUTTON))
    {
        hatchMechState = 0;
        DriverStation::ReportError("Hatch mech stage: " + std::to_string(hatchMechState));
        hatchMechStateSwitched = true;
    }
    if(hatchMechStateSwitched)
    {
        hatchMechStateSwitched = false;
        if(hatchMechState == 0)
        {
            hatchMechBottomServo->SetAngle(bottomServoUpSetpoint);
            hatchMechSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
        }
        else if(hatchMechState == 1)
        {
            hatchMechBottomServo->SetAngle(bottomServoUpSetpoint);
            hatchMechSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
        }
        else if(hatchMechState == 2)
        {
            hatchMechBottomServo->SetAngle(bottomServoDownSetpoint);
            hatchMechSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
        }
        else if(hatchMechState == 3)
        {
            hatchMechBottomServo->SetAngle(bottomServoDownSetpoint);
            hatchMechSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
        }
    }

    //Driver Lifter Controls
    if(driverStick->GetPOV() == 0) //Lifter front pneumatics down (DPAD UP)
    {
        DriverStation::ReportError("Lifter Forward Pneumatics Down");
        lifterFrontDown = true;
        lifterFrontSolenoid->Set(true);
    }
    if(driverStick->GetPOV() == 180) //Lifter back pneumatics down (DPAD DOWN)
    {
        DriverStation::ReportError("Lifter Back Pneumatics Down");
        lifterBackDown = true;
        lifterBackSolenoid->Set(true);
    }
    if(driverStick->GetPOV() == 270) //Lifter front pneumatics up (DPAD LEFT)
    {
        DriverStation::ReportError("Lifter Forward Pneumatics Up");
        lifterFrontDown = false;
        lifterFrontSolenoid->Set(false);
    }
    if(driverStick->GetPOV() == 90) //Lifter back pneumatics up (DPAD RIGHT)
    {
        DriverStation::ReportError("Lifter Back Pneumatics Up");
        lifterBackDown = false;
        lifterBackSolenoid->Set(false);
    }

    if(operatorStick->GetRawButtonPressed(JoystickButtons::A_BUTTON)) //Hatch level cargo ship (A button)
    {
        DriverStation::ReportError("Elevator Set to Ball Level CargoShip");
        setpoint = cargoShip;
        elevatorFlag = false;
        elevatorCargoLevel = true;
    }
    if(operatorStick->GetRawButtonPressed(JoystickButtons::X_BUTTON)) //Hatch rocket level 1 (X button)
    {
        DriverStation::ReportError("Elevator Set to Ball Level 1");
        setpoint = cargoRocket1;
        elevatorFlag = false;
        elevatorCargoLevel = false;
    }
    if(operatorStick->GetRawButtonPressed(JoystickButtons::B_BUTTON)) //Hatch rocket level 2 (B button)
    {
        DriverStation::ReportError("Elevator Set to Ball Level 2");
        setpoint = cargoRocket2;
        elevatorFlag = false;
        elevatorCargoLevel = false;
    }
    if(operatorStick->GetRawButtonPressed(JoystickButtons::Y_BUTTON)) //Hatch rocket level 3 (Y button)
    {
        DriverStation::ReportError("Elevator Set to Ball Level 3");
        setpoint = cargoRocket3;
        elevatorFlag = false;
        elevatorCargoLevel = false;
    }
    if(operatorStick->GetPOV() == 270 || operatorStick->GetPOV() == 180) //Hatch rocket level 1 (DPAD-LEFT)
    {
        DriverStation::ReportError("Elevator Set to Level 1");
        setpoint = -400;
        elevatorFlag = false;
        elevatorCargoLevel = false;
    }
    if(operatorStick->GetPOV() == 90) //Hatch rocket level 2 (DPAD RIGHT)
    {
        DriverStation::ReportError("Elevator Set to Hatch Level 2");
        setpoint = hatchRocket2;
        elevatorFlag = false;
        elevatorCargoLevel = false;
    }
    if(operatorStick->GetPOV() == 0) //Hatch rocket level 3 (DPAD UP)
    {
        DriverStation::ReportError("Elevator Set to Hatch Level 3");
        setpoint = hatchRocket3;
        elevatorFlag = false;
        elevatorCargoLevel = false;
    }
}

void Robot::AutonomousInit()
{

}


void Robot::AutonomousPeriodic()
{
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
