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
      bool isInverted = false;
      bool rumbleDriver = false;
      bool rumbleOperator = false;

      //Motor IDs
      const int rMotorFrontNum = 15;
      const int rMotorBackNum = 16;
      const int lMotorFrontNum = 13;
      const int lMotorBackNum = 14;
      const int elevatorMotorNum = 99;
      const int cargoIntakeMotorNum = 99;
      const int cargoLeftMotorNum = 99;
      const int cargoRightMotorNum = 99;
      const int cargoTopMotorNum = 99;

      //Solenoid IDs
      const int cargoMechLeftSolenoidNum = 99;
      const int cargoMechRightSolenoidNum = 99;
      const int hatchMechSolenoidNum = 99;

      //Servo IDs
      const int hatchMechServoNum = 99;

      //PIDs
      double pConstantDrive = 1;
      double iConstantDrive = 0;
      double dConstantDrive = 0;
      double fConstantDrive = 0;
   
      //Drive motors
      rev::CANSparkMax * lMotorFront = new rev::CANSparkMax(lMotorFrontNum, rev::CANSparkMax::MotorType::kBrushless);
      rev::CANSparkMax * lMotorBack = new rev::CANSparkMax(lMotorBackNum, rev::CANSparkMax::MotorType::kBrushless);
      rev::CANSparkMax * rMotorBack = new rev::CANSparkMax(rMotorBackNum, rev::CANSparkMax::MotorType::kBrushless);
      rev::CANSparkMax * rMotorFront = new rev::CANSparkMax(rMotorFrontNum, rev::CANSparkMax::MotorType::kBrushless);

      //Manipulator Motors
      WPI_TalonSRX * elevatorMotor = new WPI_TalonSRX(elevatorMotorNum);
      WPI_TalonSRX * cargoIntakeMotorNum = new WPI_TalonSRX(cargoIntakeMotorNum);
      WPI_TalonSRX * cargoLeftMotorNum = new WPI_TalonSRX(cargoLeftMotorNum);
      WPI_TalonSRX * cargoRightMotorNum = new WPI_TalonSRX(cargoRightMotorNum);
      WPI_TalonSRX * cargoTopMotorNum = new WPI_TalonSRX(cargoTopMotorNum);

      //Solenoids
      Solenoid * cargoMechLeftSolenoid = new Solenoid(cargoMechLeftSolenoidNum);
      Solenoid * cargoMechRightSolenoid = new Solenoid(cargoMechRightSolenoidNum);
      Solenoid * hatchMechSolenoid = new Solenoid(hatchMechSolenoidNum);

      //Servo
      Servo * hatchMechServo = new Servo(hatchMechServoNum);

      //SfDrive Object
      SFDrive_SparkMax * myRobot = new SFDrive_SparkMax(lMotorFront, rMotorFront, pConstantDrive, iConstantDrive, dConstantDrive, fConstantDrive);
      
      //Joysticks
      Joystick * driverStick = new Joystick(0);
      Joystick * operatorStick = new Joystick(1);
};
