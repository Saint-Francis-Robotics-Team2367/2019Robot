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
  SFDrive_TalonSRX(WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor, double P , double I, double D, double F );
};
