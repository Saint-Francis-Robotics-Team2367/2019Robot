/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "SFDrive.h"
#include <rev/CANSparkMax.h>

class SFDrive_SparkMax : public SFDrive {
 private:
  rev::CANSparkMax *m_leftMotor, *m_rightMotor;
  float m_leftZeroPoint, m_rightZeroPoint;
 
 public:
  SFDrive_SparkMax(rev::CANSparkMax * lMotor, rev::CANSparkMax * rMotor, double P, double I, double D, double F);

 private:
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
