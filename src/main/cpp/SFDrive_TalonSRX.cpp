/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "SFDrive_TalonSRX.h"

using namespace frc;
SFDrive_TalonSRX::SFDrive_TalonSRX(WPI_TalonSRX *lMotor, WPI_TalonSRX *rMotor, double P , double I, double D, double F ){
   m_leftMotor = lMotor;
   m_rightMotor = rMotor;
   m_P = P;
   m_I = I;
   m_D = D;
   m_F = F;
}
