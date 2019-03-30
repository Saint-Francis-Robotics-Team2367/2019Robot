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
#include <chrono>
#include <frc/Timer.h>
#include <Ultra.h>

// static class variables
void Robot::RobotInit() 
{
    //Set followers and inverts for drive motors
    //elevatorMotor->SetSensorPhase(true);
    //elevatorMotor->SetInverted(true);
    rMotorFront->SetInverted(true);
    rMotorBack->Follow(*rMotorFront, false);
    lMotorFront->SetInverted(false);
    lMotorBack->Follow(*lMotorFront, false);
    motionTimer = new frc::Timer();

    //Set current limit for drive motors
    rMotorFront->SetSmartCurrentLimit(driveMotorCurrentLimit);
    lMotorFront->SetSmartCurrentLimit(driveMotorCurrentLimit);
    rMotorBack->SetSmartCurrentLimit(driveMotorCurrentLimit);
    lMotorBack->SetSmartCurrentLimit(driveMotorCurrentLimit);
    SmartDashboard::PutNumber("Acceleration: ", profileAccel);
    SmartDashboard::PutNumber("Speed: ", profileSpeed);
    SmartDashboard::PutNumber("Distance: ", travelDistance);
    /*
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

    sender->putNumbers(); */
}

void Robot::RobotPeriodic()
{
    /*rumbleDriver = SmartDashboard::GetBoolean("Rumble Driver Joystick", rumbleDriver);
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
    } */
}

void Robot::TeleopInit()
{ /*
    cargoMechLeftSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    cargoMechRightSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    hatchMechBottomServo->SetAngle(bottomServoUpSetpoint);
    std::thread::this_thread::sleepfor(chrono::duration::seconds(1));
    hatchMechSolenoid->Set(frc::DoubleSolenoid::Value::kReverse); */

}

void Robot::TeleopPeriodic()
{ 
    //Driver has drivetrain control
    //myRobot->ModifiedAcadeDrive(driverStick->GetRawAxis(JoystickAxes::L_Y_AXIS), driverStick->GetRawAxis(JoystickAxes::R_X_AXIS));

    //Driver Granular Elevator Control
   /* if(driverStick->GetRawAxis(JoystickAxes::L_TRIGGER) > 0.05)
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
        elevatorFlag = true;
        setpoint = 0;
        elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
        elevatorMotor->SetNeutralMode(NeutralMode::Coast);
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
        cargoLeftMotor->Set(-0.8);
        cargoRightMotor->Set(-0.8);
        cargoTopMotor->Set(-0.8);
        cargoIntakeMotor->Set(-0.8);
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
    if(driverStick->GetRawButtonPressed(JoystickButtons::B_BUTTON))
    {
        hatchMechState = (hatchMechState + 3)  % 4;
        hatchMechStateSwitched = true;
        DriverStation::ReportError("Hatch mech stage: " + std::to_string(hatchMechState));
    }
    if(driverStick->GetRawButtonPressed(JoystickButtons::Y_BUTTON))
    {
        hatchMechState = (hatchMechState + 1) % 4;
        hatchMechStateSwitched = true;
        DriverStation::ReportError("Hatch mech stage: " + std::to_string(hatchMechState));
    }
    if(driverStick->GetRawButton(JoystickButtons::BACK_BUTTON) || operatorStick->GetRawButton(JoystickButtons::BACK_BUTTON))
    {
        hatchMechState = 0;
        DriverStation::ReportError("Hatch mech stage: " + std::to_string(hatchMechState));
    }
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

    if(operatorStick->GetRawButtonPressed(JoystickButtons::A_BUTTON)) //Hatch level cargo ship (A button)
    {
        DriverStation::ReportError("Elevator Set to Ball Level CargoShip");
        setpoint = cargoShip;
        elevatorMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
        elevatorFlag = true;
    }
    else if(operatorStick->GetRawButtonPressed(JoystickButtons::X_BUTTON)) //Hatch rocket level 1 (X button)
    {
        DriverStation::ReportError("Elevator Set to Ball Level 1");
        setpoint = cargoRocket1;
        elevatorFlag = true;
    }
    else if(operatorStick->GetRawButtonPressed(JoystickButtons::B_BUTTON)) //Hatch rocket level 2 (B button)
    {
        DriverStation::ReportError("Elevator Set to Ball Level 2");
        setpoint = cargoRocket2;
        elevatorFlag = true;
    }
    else if(operatorStick->GetRawButtonPressed(JoystickButtons::Y_BUTTON)) //Hatch rocket level 3 (Y button)
    {
        DriverStation::ReportError("Elevator Set to Ball Level 3");
        setpoint = cargoRocket3;
        elevatorFlag = true;
    }
    else if(operatorStick->GetPOV() == 270 || operatorStick->GetPOV() == 180) //Hatch rocket level 1 (DPAD-LEFT)
    {
        DriverStation::ReportError("Elevator Set to Level 1");
        setpoint = -400;
        elevatorFlag = true;
    }
    else if(operatorStick->GetPOV() == 90) //Hatch rocket level 2 (DPAD RIGHT)
    {
        DriverStation::ReportError("Elevator Set to Hatch Level 2");
        setpoint = hatchRocket2;
        elevatorFlag = true;
    }
    else if(operatorStick->GetPOV() == 0) //Hatch rocket level 3 (DPAD UP)
    {
        DriverStation::ReportError("Elevator Set to Hatch Level 3");
        setpoint = hatchRocket3;
        elevatorFlag = true;
    } */
}

void Robot::AutonomousInit() {
    motionTimer->Reset();
    
    //lMotorFront->GetPIDController().SetReference(RPMS, rev::ControlType::kVelocity, 0, 0);
    //rMotorFront->GetPIDController().SetReference(RPMS, rev::ControlType::kVelocity, 0, 0);

}
void Robot::placeHatchThreaded() {
    /*hatchMechBottomServo->SetAngle(bottomServoUpSetpoint); // secure hatch (functions are non blocking)
    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    hatchMechSolenoid->Set(frc::DoubleSolenoid::Value::kForward); // move hatch forward
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    hatchMechBottomServo->SetAngle(bottomServoDownSetpoint); // relase hatch
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    
    hatchMechSolenoid->Set(frc::DoubleSolenoid::Value::kReverse); // retract mechanism
    std::this_thread::sleep_for(std::chrono::seconds(1));
    */
    // N O  M U T E X
    //isHatchThreadFinished = true;
}
void Robot::AutonomousPeriodic() {
    if(moveFlag) {
        lMotorFront->GetPIDController().SetReference((profile->getValue(motionTimer) / inchesPerRev) * 60, rev::ControlType::kVelocity, 0, 0);
        rMotorFront->GetPIDController().SetReference((profile->getValue(motionTimer) / inchesPerRev) * 60, rev::ControlType::kVelocity, 0, 0);
        if(profile->getValue(motionTimer) == 0) {
            moveFlag = false;
            return;
        }
    }
    if(operatorStick->GetRawButton(JoystickButtons::LEFT_BUMPER)) {
        profileAccel = SmartDashboard::GetNumber("Acceleration: ", profileAccel);
        profileSpeed = SmartDashboard::GetNumber("Speed: ", profileSpeed);
        travelDistance = SmartDashboard::GetNumber("Distance: ", travelDistance);

        motionTimer->Start();
        profile = new motionProfiler(profileAccel, profileSpeed, travelDistance);
        moveFlag = true;
        return;
        //auton here
        //leftDist = ultra->getLeftDist();
        //rightDist = ultra->getRightDist();
        //angle = 90 - ultra->getAngle();

        //if(leftDist == 0 || rightDist == 0) {
        //    DriverStation::ReportError("One or more of the sensors cannot read a valid value. Exiting...");
       //    return;
       // }
       // arcLength = 3.1415*robotWidth*(angle/360);

       // if(leftDist > rightDist) {
      //      lMotorFront->GetPIDController().SetReference(arcLength / inchesPerRev, rev::ControlType::kPosition, 0, 0);
      //  } else if(rightDist > leftDist) {
       //     rMotorFront->GetPIDController().SetReference(arcLength / inchesPerRev, rev::ControlType::kPosition, 0, 0);
     //   }

      //  ultra->getLeftDist();
      //  ultra->getRightDist();

      //  if(std::abs(ultra->getAngle()) - 90 >= 5) {
     //       DriverStation::ReportError("Is the robot correctly aligned with the cargo ship? The sensors received: " + std::to_string(ultra->getAngle()) + " which is not within the 10 degree tolerance");
     //       return;

 
    }
    TeleopPeriodic();
    
    //myRobot->setLeftMotorSetpoint(motionTimer->Get()*travelSpeed);
    //myRobot->setRightMotorSetpoint(motionTimer->Get()*travelSpeed);
    
    //if(operatorStick->GetRawButton(JoystickButtons::LEFT_BUMPER)) { // operator left bumper for override to teleop
    //    autonOverride = true;
    //    DriverStation::ReportError("Auton has been aborted/finished. Now starting teleoperated...");
    //    myRobot->stopAutoThread();
        //hatchThread->join();
        //delete hatchThread;

    //    return; // so that the switch does not execute
    //}

    /* TODO::
    Add PIDTurn to the beginning to align with hatch *DONE
    Move forward using PIDDrive DONE
    Reverse the turning to be parallel to the cargo ship *DONE
    Start move forward sequence with CORRECTION *DONE
    Once bumper has been pressed, execute place hatch sequence *DONE
    Reverse and perform last maneuvers *DONE
    */
    /*
    switch(autonState) { // this is less bad than a sequence of if's
        case(0) :
            myRobot->setRightMotorSetpoint(40);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            DriverStation::ReportError("Alignment set."); // fix distance

            autonState++;
            break; // FALL THROUGH LOGIC 
 
        case(1) :
            if(!myRobot->isThreadFinished()) break;

            DriverStation::ReportError("Turned. Moving...");
            myRobot->setLeftMotorSetpoint(100);
            myRobot->setRightMotorSetpoint(100);

            std::this_thread::sleep_for(std::chrono::seconds(1));
            autonState++;
            break;
        
        case(2) : 
            if(!myRobot->isThreadFinished()) break;
            myRobot->setLeftMotorPosition(40);
            DriverStation::ReportError("Correcting angle...");
            //myRobot->PIDTurn(startingAngle, 0, maxVel, 0, true); // fix distance
        
            std::this_thread::sleep_for(std::chrono::seconds(1));
            autonState++;
            break;
        
        case(3) :
            if(!myRobot->isThreadFinished()) break;
            DriverStation::ReportError("Started correction stage. Move joystick to correct alignment...");

            myRobot->setLeftMotorPosition(0); // reset encoders
            myRobot->setRightMotorPosition(0);

            autonState++;
            break;
        case(4) : 
            if(driverStick->GetRawButton(JoystickButtons::RIGHT_BUMPER)) { // hatch placement check
                DriverStation::ReportError("Placing Hatch.");
                hatchThread = new std::thread(&Robot::placeHatchThreaded, this);
                autonState++;
                break;
            }

            xCorrect = driverStick->GetRawAxis(JoystickAxes::R_X_AXIS); // Inputs for correction
            
            xCorrect *= correctionSensitivity; // this should be adjusted to driver preference

            if(xCorrect < 0) {
                rSetpointCorrect += std::abs(xCorrect); // if the joystick is to the left, add to the right
            } else if(xCorrect > 0) {
                lSetpointCorrect += std::abs(xCorrect); // if the joystick is to the right, add to the left
            }

            /* ---------
            We do NOT want to use motion profiling here because we want the 
            speed to be as predictable as possible for the operator making the corrections.

            If the speed changed over time, the operator would have to make less correction during the slow parts,
            and more during the fast parts, making it hard to be accurate

            We use a timer to calculate the setpoint based on the correctionPeriodVelocity and add to it the 
            setpoint correction as determined by the joystick position and the correctionSensitivity
            --------- 

            myRobot->setRightMotorSetpoint(rSetpointCorrect);
            myRobot->setLeftMotorSetpoint(lSetpointCorrect);
            
            break; 

        case(5) :
            // hatch threading finished conditional
            if(!Robot::isHatchThreadFinished) break;

            DriverStation::ReportError("Reversing...");
            myRobot->PIDDriveThread(reverseDistance, maxVel, 0, true); // fix distance
            autonState++;
            break;
            
      /*  case(6) :
            // turn 
            if(!myRobot->isThreadFinished()) break;

            DriverStation::ReportError("Turning...");
           // myRobot->PIDTurnThread(distanceToFeederStation, 0, maxVel, 0, true); // 
            autonState++;
            break;

        case(7) : 
            // final move
            if(!myRobot->isThreadFinished()) break;

            DriverStation::ReportError("Performing Final Move...");
            myRobot->PIDDriveThread(20, maxVel, 0, true);
            autonState++;
            break; */ 
    //}   


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
