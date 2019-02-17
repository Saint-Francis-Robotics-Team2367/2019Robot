/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <ctre/Phoenix.h>
#include <SFDrive.h>

#pragma once

class SFDrive_TalonSRX : public SFDrive {

 private:
  WPI_TalonSRX * m_leftMotor, *m_rightMotor;

 public:
  SFDrive_TalonSRX(WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor, double P , double I, double D, double F);
  virtual void setLeftMotor(double value);
  virtual void setRightMotor(double value);
  virtual void setLeftMotorPosition(int ticks);
  virtual void setRightMotorPosition(int ticks);
  virtual void setLeftMotorSetpoint(int ticks);
  virtual void setRightMotorSetpoint(int ticks);
  virtual void setP(double value);
  virtual void setI(double value);
  virtual void setD(double value);
};
