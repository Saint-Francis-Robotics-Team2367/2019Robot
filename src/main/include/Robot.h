/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <ctre/Phoenix.h>
#include "SFDrive_TalonSRX.h"
#include "SFDrive_SparkMax.h"
#include <frc/Joystick.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/DriverStation.h>
#include <frc/Solenoid.h>
#include <frc/Servo.h>
#include <SmartSender.h>

using namespace frc;

class Robot : public frc::TimedRobot {
   public:
      void RobotInit() override;
      void RobotPeriodic() override;
      void AutonomousInit() override;
      void AutonomousPeriodic() override;
      void TeleopInit() override;
      void TeleopPeriodic() override;
      void TestPeriodic() override;
      void DisabledInit() override;
      void ConfigPIDS();
      
      //Control variables
      bool singleController = false;
      bool operatorInCargoMode = true; //false = hatch panel mode
      bool rumbleDriver = false;
      bool rumbleOperator = false;
      int driveMotorCurrentLimit = 30;
      bool outputtingCargo = false;
      double outputtingCargoStartTime = 0;
      double elevatorGranularControlMultiplier = 0.3;
      int hatchMechState = 0;
      bool hatchMechStateSwitched = true;
      //THESE ASSUMPTIONS ARE PROBABLY INCORRECT
      double topServoUpSetpoint = 0;
      double topServoDownSetpoint = 0;
      double bottomServoUpSetpoint = 0;
      double bottomServoDownSetpoint = 0;

      //Motor IDs
      const int rMotorFrontNum = 15;
      const int rMotorBackNum = 16;
      const int lMotorFrontNum = 2;
      const int lMotorBackNum = 3;
      //THESE ASSUMPTIONS ARE PROBABLY INCORRECT
      const int elevatorMotorNum = 9;
      const int cargoIntakeMotorNum = 11;
      const int cargoLeftMotorNum = 99;
      const int cargoRightMotorNum = 99;
      const int cargoTopMotorNum = 99;

      //Solenoid IDs
      //THESE ASSUMPTIONS ARE PROBABLY INCORRECT
      const int cargoMechLeftSolenoidNum = 0;
      const int cargoMechRightSolenoidNum = 1;
      const int hatchMechSolenoidNum = 99;

      //Servo IDs
      //THESE ASSUMPTIONS ARE PROBABLY INCORRECT
      const int hatchMechTopServoNum = 0;
      const int hatchMechBottomServoNum = 0;

      //Drive Constants
      const double pConstantDrive = 1;
      const double iConstantDrive = 0;
      const double dConstantDrive = 0;
      const double fConstantDrive = 0;

      //Elevator Constants
      //THESE ASSUMPTIONS ARE PROBABLY INCORRECT
      const int cargoRocket1 = -10;
      const int cargoRocket2 = -10;
      const int cargoRocket3 = -10;
      const int cargoShip = -10;
      const int hatchRocket1 = -10;
      const int hatchRocket2 = -10;
      const int hatchRocket3 = -10;
      const double pConstantElevator = 1;
      const double iConstantElevator = 0;
      const double dConstantElevator = 0;

      //Test values
      double setpoint = -8000;

      //Drive motors
      rev::CANSparkMax * lMotorFront = new rev::CANSparkMax(lMotorFrontNum, rev::CANSparkMax::MotorType::kBrushless);
      rev::CANSparkMax * lMotorBack = new rev::CANSparkMax(lMotorBackNum, rev::CANSparkMax::MotorType::kBrushless);
      rev::CANSparkMax * rMotorBack = new rev::CANSparkMax(rMotorBackNum, rev::CANSparkMax::MotorType::kBrushless);
      rev::CANSparkMax * rMotorFront = new rev::CANSparkMax(rMotorFrontNum, rev::CANSparkMax::MotorType::kBrushless);

      //Manipulator Motors
      WPI_TalonSRX * elevatorMotor = new WPI_TalonSRX(elevatorMotorNum);
      WPI_TalonSRX * cargoIntakeMotor = new WPI_TalonSRX(cargoIntakeMotorNum);
      WPI_TalonSRX * cargoLeftMotor = new WPI_TalonSRX(cargoLeftMotorNum);
      WPI_TalonSRX * cargoRightMotor = new WPI_TalonSRX(cargoRightMotorNum);
      WPI_TalonSRX * cargoTopMotor = new WPI_TalonSRX(cargoTopMotorNum);

      //Solenoids
      Solenoid * cargoMechLeftSolenoid = new Solenoid(cargoMechLeftSolenoidNum);
      Solenoid * cargoMechRightSolenoid = new Solenoid(cargoMechRightSolenoidNum);
      Solenoid * hatchMechSolenoid = new Solenoid(hatchMechSolenoidNum);

      //Servo
      Servo * hatchMechTopServo = new Servo(hatchMechTopServoNum);
      Servo * hatchMechBottomServo = new Servo(hatchMechBottomServoNum);

      //SfDrive Object
      SFDrive_SparkMax * myRobot = new SFDrive_SparkMax(lMotorFront, rMotorFront, pConstantDrive, iConstantDrive, dConstantDrive, fConstantDrive);
      
      //SmartSender Object
      SmartSender * sender = new SmartSender();

      //Joysticks
      Joystick * driverStick = new Joystick(0);
      Joystick * operatorStick = new Joystick(1);
};
