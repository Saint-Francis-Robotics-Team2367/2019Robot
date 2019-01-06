/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "SFDrive.h"

SFDrive::SFDrive(WPI_TalonSRX *lMotor, WPI_TalonSRX *rMotor)
{
   m_leftMotor = lMotor;
   m_rightMotor = rMotor;
}
