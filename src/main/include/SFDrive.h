/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

class SFDrive {
  protected:
   double m_deadband = 0.08;
   const float m_wheelCircumference = 6 * 3.14;
   const float m_ticksPerRev = 1024 * 4;
   const float m_canTimeout = 0;
   float m_currVelocity = 0;
   float m_maxAccl = 8000;
   const float m_minDecelVel = 27 / m_wheelCircumference * m_ticksPerRev;
   double m_P;
   double m_I;
   double m_D;
   double m_F;
   const float m_wheelTrack = 24;
   const double m_timeoutMs = 0;

 public:
  void ArcadeDrive(double xSpeed, double zRotation);
  bool PIDDrive(float inches, float maxVel, float timeout = 4, bool ZeroVelocityAtEnd = true);
  bool PIDTurn(float degreesClockwise, float radius, float maxVel, float timeout = 4, bool ZeroVelocityAtEnd = true);
  void disableP();
  void enableP();
  void initPID();
  void setAccel(float);

 protected:
  virtual void setLeftMotor(double value) = 0;
  virtual void setRightMotor(double value) = 0;
  virtual void setLeftMotorPosition(int ticks) = 0;
  virtual void setRightMotorPosition(int ticks) = 0;
  virtual void setLeftMotorSetpoint(int ticks) = 0;
  virtual void setRightMotorSetpoint(int ticks) = 0;
  virtual void setP(double value) = 0;
  virtual void setI(double value) = 0;
  virtual void setD(double value) = 0;
};
