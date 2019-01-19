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
#include <frc/Joystick.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/DriverStation.h>
#include <rev/CANSparkMax.h>

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
      void ConfigPIDS();
      const int joystickNum = 0;
      const int rMotorFrontNum = 5;
      const int rMotorBackNum = 4;
      const int lMotorFrontNum = 3;
      const int lMotorBackNum = 2;
      const double TICKS_PER_INCH = 217.3;
      double pConstantDrive = 1;
      double iConstantDrive = 0;
      double dConstantDrive = 10;
      double fConstantDrive = 0;
      double maxDriveMotorCurrent = 30;
      int checkTimeout = 0;
      int timeOut = 100;
      double rumbleMultiplier = 1.0/8.0;
      double rumbleDeadzone = 0.5;
      double scale = 1;

   private:
      WPI_TalonSRX * lMotorFront = new WPI_TalonSRX(lMotorFrontNum);
      WPI_TalonSRX * lMotorBack = new WPI_TalonSRX(lMotorBackNum);
      WPI_TalonSRX * rMotorFront = new WPI_TalonSRX(rMotorFrontNum);
      WPI_TalonSRX * rMotorBack = new WPI_TalonSRX(rMotorBackNum);
      rev::CANSparkMax * spark = new rev::CANSparkMax(0, rev::CANSparkMax::MotorType::kBrushless);
      SFDrive_TalonSRX * myRobot = new SFDrive_TalonSRX(lMotorFront, rMotorFront, pConstantDrive, iConstantDrive, dConstantDrive, fConstantDrive);
      Joystick *stick = new Joystick(joystickNum);
      BuiltInAccelerometer accelerometer;
};
