/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "SFDrive_TalonSRX.h"
#include <frc/Timer.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include <frc/SmartDashboard/SmartDashboard.h>

using namespace frc;
SFDrive_TalonSRX::SFDrive_TalonSRX(WPI_TalonSRX *lMotor, WPI_TalonSRX *rMotor, double P , double I, double D, double F ){
   m_leftMotor = lMotor;
   m_rightMotor = rMotor;
   m_P = P;
   m_I = I;
   m_D = D;
   m_F = F;
}

void SFDrive_TalonSRX::ArcadeDrive(double xSpeed, double zRotation){
   double leftMotorOutput;
   double rightMotorOutput;

   if (fabs(xSpeed) <= m_deadband)
      xSpeed = 0;
   if (fabs(zRotation) <= m_deadband)
      zRotation = 0;

   double maxInput = std::copysign(std::max(std::abs(xSpeed), std::abs(zRotation)), xSpeed);

   if (xSpeed >= 0.0)
   {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0)
      {
         leftMotorOutput = maxInput;
         rightMotorOutput = xSpeed - zRotation;
      }
      else
      {
         leftMotorOutput = xSpeed + zRotation;
         rightMotorOutput = maxInput;
      }
   }
   else
   {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0)
      {
         leftMotorOutput = xSpeed + zRotation;
         rightMotorOutput = maxInput;
      }
      else
      {
         leftMotorOutput = maxInput;
         rightMotorOutput = xSpeed - zRotation;
      }
   }
   m_leftMotor->Set(leftMotorOutput);
   m_rightMotor->Set(-rightMotorOutput);
}

bool SFDrive_TalonSRX::PIDDrive(float inches, float maxVel, float timeout, bool ZeroVelocityAtEnd){

   int setPoint = 0;
   double startTime, currStepTime, lastStepTime, deltaTime;

   //convert from inches to encoder ticks
   float endPoint = abs(inches) / m_wheelCircumference * m_ticksPerRev;
   int maxVelDelta = maxVel / m_wheelCircumference * m_ticksPerRev;

   //zero encoder
   disableP();
   m_leftMotor->SetSelectedSensorPosition(0, 0, m_canTimeout);
   m_rightMotor->SetSelectedSensorPosition(0, 0, m_canTimeout);
   m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, 0);
   m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, 0);
   enableP();

   if (ZeroVelocityAtEnd)
   {
      startTime = lastStepTime = Timer().GetFPGATimestamp();
      while ((int) setPoint < (int) endPoint && startTime + timeout > lastStepTime)
      {
         //handle timing
         currStepTime = Timer().GetFPGATimestamp();
         deltaTime = currStepTime - lastStepTime;
         lastStepTime = currStepTime;

         if (endPoint - setPoint < (m_currVelocity * m_currVelocity) / (2 * m_maxAccl)) //check if you decelerated at the current speed
         {																		  //would you hit 0 before you reached the end point
            m_currVelocity -= m_maxAccl * deltaTime;
            if (m_currVelocity < m_minDecelVel)
               m_currVelocity = m_minDecelVel;
         }
         else																			//otherwise accelerate
         {
            m_currVelocity += m_maxAccl * deltaTime;
            if (m_currVelocity > maxVelDelta)
               m_currVelocity = maxVelDelta;
         }

         setPoint += m_currVelocity * deltaTime;
         if (setPoint > endPoint)
            setPoint = endPoint;

         SmartDashboard::PutNumber("Left Encoder", endPoint - setPoint);
         SmartDashboard::PutNumber("Right Encoder", endPoint - setPoint);

         m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setPoint, inches) * -1);
         m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setPoint, inches));
      }
   }
   else
   {
      startTime = lastStepTime = Timer().GetFPGATimestamp();
      while ((int) setPoint < (int) endPoint && startTime + timeout > lastStepTime)
      {
         //handle timing
         currStepTime = Timer().GetFPGATimestamp();
         deltaTime = currStepTime - lastStepTime;
         lastStepTime = currStepTime;

         m_currVelocity += m_maxAccl * deltaTime;
         if (m_currVelocity > maxVelDelta)
            m_currVelocity = maxVelDelta;
         setPoint += m_currVelocity * deltaTime;
         if (setPoint > endPoint)
            setPoint = endPoint;

         m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setPoint, inches) * -1);
         m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setPoint, inches));
      }
   }

   if (lastStepTime > startTime + timeout) //simple error check, did we finish the motion before we ran out of time
   {
      return false;
   }
   else
   {
      return true;
   }
}

bool SFDrive_TalonSRX::PIDTurn(float degreesClockwise, float radius, float maxVel, float timeout, bool ZeroVelocityAtEnd){

   int setPoint = 0, endPoint, innerChordLen;
   float endAngle = abs(degreesClockwise);
   int maxVelDelta = maxVel / m_wheelCircumference * m_ticksPerRev;
   double startTime, currStepTime, lastStepTime, deltaTime;

   //zero encoder
   disableP();
   m_leftMotor->SetSelectedSensorPosition(0, 0, m_canTimeout);
   m_rightMotor->SetSelectedSensorPosition(0, 0, m_canTimeout);
   m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, 0);
   m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, 0);
   enableP();

   if (radius < m_wheelTrack / 2)
      radius = m_wheelTrack / 2.0f;
   endPoint = (((endAngle / 360.0f) * ((radius + m_wheelTrack / 2.0f) * 2 * M_PI)) / m_wheelCircumference) * m_ticksPerRev;
   innerChordLen = (((endAngle / 360.0f) * ((radius - m_wheelTrack / 2.0f) * 2 * M_PI)) / m_wheelCircumference) * m_ticksPerRev;

   startTime = lastStepTime = Timer().GetFPGATimestamp();

   if (ZeroVelocityAtEnd)
   {
      startTime = lastStepTime = Timer().GetFPGATimestamp();
      while ((int) setPoint < (int) endPoint && startTime + timeout > lastStepTime)
      {
         //handle timing
         currStepTime = Timer().GetFPGATimestamp();
         deltaTime = currStepTime - lastStepTime;
         lastStepTime = currStepTime;

         if (endPoint - setPoint < (m_currVelocity * m_currVelocity) / (2 * m_maxAccl)) //check if you decelerated at the current speed
         {																		  //would you hit 0 before you reached the end point
            m_currVelocity -= m_maxAccl * deltaTime;
            if (m_currVelocity < m_minDecelVel)
               m_currVelocity = m_minDecelVel;
         }
         else																			//otherwise accelerate
         {
            m_currVelocity += m_maxAccl * deltaTime;
            if (m_currVelocity > maxVelDelta)
               m_currVelocity = maxVelDelta;
         }

         setPoint += m_currVelocity * deltaTime;
         if (setPoint > endPoint)
            setPoint = endPoint;
         int innerSet = ((float) setPoint / (float) endPoint) * innerChordLen;

         if (degreesClockwise > 0)
         {
            m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, setPoint * -1);
            m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, innerSet);
         }
         else
         {
            m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, innerSet * -1);
            m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, setPoint);
         }
      }
   }
   else
   {
      while ((int) setPoint < (int) endPoint && startTime + timeout > lastStepTime)
      {
         //handle timing
         currStepTime = Timer().GetFPGATimestamp();
         deltaTime = currStepTime - lastStepTime;
         lastStepTime = currStepTime;

         //move
         m_currVelocity += m_maxAccl * deltaTime;
         if (m_currVelocity > maxVelDelta)
            m_currVelocity = maxVelDelta;
         setPoint += m_currVelocity * deltaTime;
         if (setPoint > endPoint)
            setPoint = endPoint;
         int innerSet = ((float) setPoint / (float) endPoint) * innerChordLen;

         if (degreesClockwise > 0)
         {
            m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, setPoint * -1);
            m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, innerSet);
         }
         else
         {
            m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, innerSet * -1);
            m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, setPoint);
         }
      }
   }

   if (lastStepTime > startTime + timeout) //simple error check, did we finish the motion before we ran out of time
   {
      return false;
   }
   else
   {
      return true;
   }
}


void SFDrive_TalonSRX::initPID(){
   m_leftMotor->Config_kP(0, m_P, m_canTimeout);
   m_leftMotor->Config_kI(0, m_I, m_canTimeout);
   m_leftMotor->Config_kD(0, m_D, m_canTimeout);

   m_rightMotor->Config_kP(0, m_P, m_canTimeout);
   m_rightMotor->Config_kI(0, m_I, m_canTimeout);
   m_rightMotor->Config_kD(0, m_D, m_canTimeout);
}

void SFDrive_TalonSRX::enableP(){
   m_leftMotor->Config_kP(0, m_P, m_canTimeout);
   m_rightMotor->Config_kP(0, m_P, m_canTimeout);
}

void SFDrive_TalonSRX::disableP(){
   m_leftMotor->Config_kP(0, 0, m_canTimeout);
   m_rightMotor->Config_kP(0, 0, m_canTimeout);
}

void SFDrive_TalonSRX::setAccel(float accl){
   m_maxAccl = accl;
}
