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
#include "SFDrive_SparkMax.h"
#include <frc/Joystick.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/DriverStation.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
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
      bool operatorInCargoMode = true; //false = hatch panel mode
      bool rumbleDriver = false;
      bool rumbleOperator = false;
      bool outputtingCargo = false;
      double outputtingCargoStartTime = 0;
      double elevatorGranularControlMultiplier = 320;
      int hatchMechState = 0;
      bool hatchMechStateSwitched = true;
      int setpoint = 0;
      bool elevatorFlag = false;
      bool elevatorCargoLevel = false;
      int hatchMechSetpoint = 0;
      double bottomServoUpSetpoint = 67;
      double bottomServoDownSetpoint = 0;
      bool lifterFrontDown = false;
      bool lifterBackDown = false;
      int hatchMechStage = 0;
      double elevatorCurrentTicks = 0;

      //Motor IDs
      const int rMotorFrontNum = 13;
      const int rMotorBackNum = 16;
      const int lMotorFrontNum = 2;
      const int lMotorBackNum = 3;
      const int elevatorMotorNum = 1;
      const int cargoIntakeMotorNum = 15;
      const int cargoLeftMotorNum = 10;
      const int cargoRightMotorNum = 14;
      const int cargoTopMotorNum = 12;

      //Solenoid IDs
      const int cargoMechLeftSolenoidNum = 4;
      const int cargoMechRightSolenoidNum = 2;
      const int hatchMechSolenoidNum = 0;
      const int lifterFrontSolenoidNum = 0;
      const int lifterBackSolenoidNum = 2;
      const int PCMOneNum = 20;
      const int PCMTwoNum = 0;

      //Servo IDs
      const int hatchMechBottomServoNum = 4;

      //Drive Constants
      const double pConstantDrive = 1;
      const double iConstantDrive = 0;
      const double dConstantDrive = 0;
      const double fConstantDrive = 0;
      const int driveMotorCurrentLimit = 40;

      //Elevator Constants
      const int cargoRocket1 = -10135;
      const int cargoRocket2 = -24271;
      const int cargoRocket3 = -37080;
      const int cargoShip = -15366;
      const int hatchRocket2 = -15366;
      const int hatchRocket3 = -29099;
      const double pConstantElevator = 1;
      const double iConstantElevator = 0;
      const double dConstantElevator = 0;
      const int elevatorPeakMotorCurrentLimit = 40;
      const int elevatorContinuousMotorCurrentLimit = 30;
      const int elevatorPeakMotorCurrentLimitDuration = 500; //in milliseconds

      //Drive motors
      rev::CANSparkMax * lMotorFront = new rev::CANSparkMax(lMotorFrontNum, rev::CANSparkMax::MotorType::kBrushless);
      rev::CANSparkMax * lMotorBack = new rev::CANSparkMax(lMotorBackNum, rev::CANSparkMax::MotorType::kBrushless);
      rev::CANSparkMax * rMotorBack = new rev::CANSparkMax(rMotorBackNum, rev::CANSparkMax::MotorType::kBrushless);
      rev::CANSparkMax * rMotorFront = new rev::CANSparkMax(rMotorFrontNum, rev::CANSparkMax::MotorType::kBrushless);

      //Manipulator Motors
      WPI_TalonSRX * elevatorMotor = new WPI_TalonSRX(elevatorMotorNum);
      WPI_TalonSRX * cargoIntakeMotor = new WPI_TalonSRX(cargoIntakeMotorNum);
      WPI_VictorSPX * cargoLeftMotor = new WPI_VictorSPX(cargoLeftMotorNum);
      WPI_VictorSPX * cargoRightMotor = new WPI_VictorSPX(cargoRightMotorNum);
      WPI_VictorSPX * cargoTopMotor = new WPI_VictorSPX(cargoTopMotorNum);

      //Solenoids
      DoubleSolenoid * cargoMechLeftSolenoid = new DoubleSolenoid(PCMOneNum, cargoMechLeftSolenoidNum, cargoMechLeftSolenoidNum + 1);
      DoubleSolenoid * cargoMechRightSolenoid = new DoubleSolenoid(PCMOneNum, cargoMechRightSolenoidNum, cargoMechRightSolenoidNum + 1);
      DoubleSolenoid * hatchMechSolenoid = new DoubleSolenoid(PCMOneNum, hatchMechSolenoidNum, hatchMechSolenoidNum + 1);
      DoubleSolenoid * lifterFrontSolenoid = new DoubleSolenoid(PCMTwoNum, lifterFrontSolenoidNum, lifterFrontSolenoidNum + 1);
      DoubleSolenoid * lifterBackSolenoid = new DoubleSolenoid(PCMTwoNum, lifterBackSolenoidNum, lifterBackSolenoidNum + 1);

      //Servo
      Servo * hatchMechBottomServo = new Servo(hatchMechBottomServoNum);

      //SfDrive Object
      SFDrive_SparkMax * myRobot = new SFDrive_SparkMax(lMotorFront, rMotorFront, pConstantDrive, iConstantDrive, dConstantDrive, fConstantDrive);
      
      //SmartSender Object
      SmartSender * sender = new SmartSender();

      //Joysticks
      Joystick * driverStick = new Joystick(0);
      Joystick * operatorStick = new Joystick(1);
};
