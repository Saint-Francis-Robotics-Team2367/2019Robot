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
      const int joystickNum = 0;
      const int rMotorFrontNum = 5;
      const int rMotorBackNum = 4;
      const int lMotorFrontNum = 3;
      const int lMotorBackNum = 2;
      const double TICKS_PER_INCH = 217.3;
      double pConstantDrive = 1;
      double iConstantDrive = 0;
      double dConstantDrive = 0;
      double fConstantDrive = 0;
      double maxDriveMotorCurrent = 30;
      int checkTimeout = 0;
      int timeOut = 100;
      double rumbleMultiplier = 1.0/8.0;
      double rumbleDeadzone = 0.5;
      double scale = 1;
      double turning = 0;

   private:
      rev::CANSparkMax * lMotorFront = new rev::CANSparkMax(1, rev::CANSparkMax::MotorType::kBrushless);
      rev::CANSparkMax * lMotorBack = new rev::CANSparkMax(0, rev::CANSparkMax::MotorType::kBrushless);
      rev::CANSparkMax * rMotorBack = new rev::CANSparkMax(3, rev::CANSparkMax::MotorType::kBrushless);
      rev::CANSparkMax * rMotorFront = new rev::CANSparkMax(2, rev::CANSparkMax::MotorType::kBrushless);
      SFDrive_SparkMax * myRobot = new SFDrive_SparkMax(lMotorFront, rMotorFront, pConstantDrive, iConstantDrive, dConstantDrive, fConstantDrive);
      Joystick *stick = new Joystick(joystickNum);
      BuiltInAccelerometer accelerometer;
};
