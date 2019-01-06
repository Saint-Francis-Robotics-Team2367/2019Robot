/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <ctre/Phoenix.h>

#pragma once

class SFDrive {

 private:
   WPI_TalonSRX * m_leftMotor, *m_rightMotor;
   double m_deadband = 0.08;
   const float m_wheelCircumference = 6 * 3.14;
   const float m_ticksPerRev = 1024 * 4;
   const float m_canTimeout = 0;
   float m_currVelocity = 0;
   float m_maxAccl = 8000;
   const float m_minDecelVel = 27 / m_wheelCircumference * m_ticksPerRev;
   const float m_P = 1;
   const float m_I = 0;
   const float m_D = 10;
   const float m_wheelTrack = 24;

 public:
  SFDrive(WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor);
  void ArcadeDrive(double xSpeed, double zRotation);
  bool PIDDrive(float inches, float maxVel, float timeout = 4, bool ZeroVelocityAtEnd = true);
  bool PIDTurn(float degreesClockwise, float radius, float maxVel, float timeout = 4, bool ZeroVelocityAtEnd = true);
  void disableP();
  void enableP();
  void initPID();
};
