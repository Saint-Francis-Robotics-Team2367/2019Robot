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
   m_ticksPerRev = 4096;
   m_minDecelVel = 27 / m_wheelCircumference * m_ticksPerRev;
}

void SFDrive_TalonSRX::setLeftMotor(double value)
{
   m_leftMotor->Set(value);
}
void SFDrive_TalonSRX::setRightMotor(double value)
{
   m_rightMotor->Set(value);
}
void SFDrive_TalonSRX::setLeftMotorPosition(int ticks)
{
   m_leftMotor->SetSelectedSensorPosition(ticks, 0, m_timeoutMs);
}
void SFDrive_TalonSRX::setRightMotorPosition(int ticks)
{
   m_rightMotor->SetSelectedSensorPosition(ticks, 0, m_timeoutMs);
}
void SFDrive_TalonSRX::setLeftMotorSetpoint(int ticks)
{
   m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, ticks);
}
void SFDrive_TalonSRX::setRightMotorSetpoint(int ticks)
{
   m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, ticks);
}
void SFDrive_TalonSRX::setP(double value)
{
   m_leftMotor->Config_kP(0, value, m_timeoutMs);
   m_rightMotor->Config_kP(0, value, m_timeoutMs);
}
void SFDrive_TalonSRX::setI(double value)
{
   m_leftMotor->Config_kI(0, value, m_timeoutMs);
   m_rightMotor->Config_kI(0, value, m_timeoutMs);
}
void SFDrive_TalonSRX::setD(double value)
{
   m_leftMotor->Config_kD(0, value, m_timeoutMs);
   m_rightMotor->Config_kD(0, value, m_timeoutMs);
}
