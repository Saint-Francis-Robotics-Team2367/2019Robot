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
   m_ticksPerRev = 42;
   m_leftZeroPoint = m_leftMotor->GetEncoder().GetPosition() * 42;
   m_rightZeroPoint = m_rightMotor->GetEncoder().GetPosition() * 42;
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
    m_leftZeroPoint = m_leftMotor->GetEncoder().GetPosition() * 42 - ticks;
}

void SFDrive_SparkMax::setRightMotorPosition(int ticks)
{
    m_rightZeroPoint = m_rightMotor->GetEncoder().GetPosition() * 42 - ticks;
}

void SFDrive_SparkMax::setLeftMotorSetpoint(int ticks)
{
    m_leftMotor->PIDWrite(ticks + m_leftZeroPoint);
}

void SFDrive_SparkMax::setRightMotorSetpoint(int ticks)
{
    m_rightMotor->PIDWrite(ticks + m_rightZeroPoint);
}

void SFDrive_SparkMax::setP(double value)
{
    m_leftMotor->SetParameter(rev::CANSparkMaxLowLevel::ConfigParameter::kP_0, value);
    m_rightMotor->SetParameter(rev::CANSparkMaxLowLevel::ConfigParameter::kP_0, value);
}

void SFDrive_SparkMax::setI(double value)
{
    m_leftMotor->SetParameter(rev::CANSparkMaxLowLevel::ConfigParameter::kI_0, value);
    m_rightMotor->SetParameter(rev::CANSparkMaxLowLevel::ConfigParameter::kI_0, value);
}

void SFDrive_SparkMax::setD(double value)
{
    m_leftMotor->SetParameter(rev::CANSparkMaxLowLevel::ConfigParameter::kD_0, value);
    m_rightMotor->SetParameter(rev::CANSparkMaxLowLevel::ConfigParameter::kD_0, value);
}
