/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "SFDrive_SparkMax.h"

SFDrive_SparkMax::SFDrive_SparkMax(rev::CANSparkMax * lMotor, rev::CANSparkMax * rMotor, double P, double I, double D, double F) 
{  
   m_leftMotor = lMotor;
   m_rightMotor = rMotor;
   m_P = P;
   m_I = I;
   m_D = D;
   m_F = F;
   initPID();
   m_ticksPerRev = 42;
   m_minDecelVel = 27 / m_wheelCircumference * m_ticksPerRev;
}

void SFDrive_SparkMax::setLeftMotor(double value)
{
    m_leftMotor->Set(value);
}

void SFDrive_SparkMax::setRightMotor(double value)
{
    m_rightMotor->Set(value);
}

void SFDrive_SparkMax::setLeftMotorPosition(int ticks)
{
    m_leftMotor->GetEncoder().SetPosition(ticks);
}

void SFDrive_SparkMax::setRightMotorPosition(int ticks)
{
    m_rightMotor->GetEncoder().SetPosition(ticks);
}

void SFDrive_SparkMax::setLeftMotorSetpoint(int ticks)
{
    m_leftMotor->GetPIDController().SetReference(ticks / m_ticksPerRev, rev::ControlType::kPosition, 0, 0);
}

void SFDrive_SparkMax::setRightMotorSetpoint(int ticks)
{
    m_rightMotor->GetPIDController().SetReference(ticks / m_ticksPerRev, rev::ControlType::kPosition, 0, 0);
}

void SFDrive_SparkMax::setP(double value)
{
    m_leftMotor->GetPIDController().SetP(value, 0);
    m_rightMotor->GetPIDController().SetP(value, 0);
}

void SFDrive_SparkMax::setI(double value)
{
    m_leftMotor->GetPIDController().SetI(value, 0);
    m_rightMotor->GetPIDController().SetI(value, 0);
}

void SFDrive_SparkMax::setD(double value)
{
    m_leftMotor->GetPIDController().SetD(value, 0);
    m_rightMotor->GetPIDController().SetD(value, 0);
}
