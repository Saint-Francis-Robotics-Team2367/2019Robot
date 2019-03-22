/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "SFDrive.h"
#include <frc/Timer.h>
#include <math.h>

using namespace frc;

void SFDrive::ModifiedAcadeDrive(double xSpeed, double zRotation)
{
   if(abs(xSpeed) < m_deadband)
   {
      xSpeed = 0;
   }
   else if(abs(xSpeed) < (1 - m_deadband) * m_thresholdPercentage + m_deadband)
   {
      xSpeed = std::copysign(m_lowSpeedControlMultiplier * (abs(xSpeed)  - m_deadband), xSpeed);
   }
   else
   {
      xSpeed = std::copysign(m_highSpeedControlMultiplier * abs(xSpeed) + 1 - m_highSpeedControlMultiplier, xSpeed);
   }

   if(abs(zRotation) < m_deadband)
   {
      zRotation = 0;
   }
   else if(abs(zRotation) < (1 - m_deadband) * m_thresholdPercentage + m_deadband)
   {
      zRotation = std::copysign(m_lowSpeedControlMultiplier * (abs(zRotation)  - m_deadband), zRotation);
   }
   else
   {
      zRotation = std::copysign(m_highSpeedControlMultiplier * abs(zRotation) + 1 - m_highSpeedControlMultiplier, zRotation);
   }

   ArcadeDrive(xSpeed, zRotation);
}

void SFDrive::ArcadeDrive(double xSpeed, double zRotation){
   double leftMotorOutput;
   double rightMotorOutput;

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
   setLeftMotor(leftMotorOutput);
   setRightMotor(rightMotorOutput);
}

bool SFDrive::PIDDrive(float inches, float maxVel, float timeout, bool ZeroVelocityAtEnd){

   int setPoint = 0;
   double startTime, currStepTime, lastStepTime, deltaTime;

   //convert from inches to encoder ticks
   float endPoint = abs(inches) / m_wheelCircumference * m_ticksPerRev;
   int maxVelDelta = maxVel / m_wheelCircumference * m_ticksPerRev;

   //zero encoder
   disableP();
   setLeftMotorPosition(0);
   setRightMotorPosition(0);
   enableP();

   if (ZeroVelocityAtEnd)
   {
      startTime = lastStepTime = Timer().GetFPGATimestamp();
      while ((int) setPoint < (int) endPoint && startTime + timeout > lastStepTime)
      {
         if(stopThread)
         {
            stopThread = false;
            break;
         }
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

         setLeftMotorSetpoint(std::copysign(setPoint, inches) * -1);
         setRightMotorSetpoint(std::copysign(setPoint, inches));
      }
   }
   else
   {
      startTime = lastStepTime = Timer().GetFPGATimestamp();
      while ((int) setPoint < (int) endPoint && startTime + timeout > lastStepTime)
      {
         if(stopThread)
         {
            stopThread = false;
            break;
         }
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

         setLeftMotorSetpoint(std::copysign(setPoint, inches) * -1);
         setRightMotorSetpoint(std::copysign(setPoint, inches));
      }
   }

   threadFinished = true;

   if (lastStepTime > startTime + timeout) //simple error check, did we finish the motion before we ran out of time
   {
      return false;
   }
   else
   {
      return true;
   }
}

bool SFDrive::PIDDriveThread(float inches, float maxVel, float timeout, bool ZeroVelocityAtEnd){
   stopThread = false; //Idiot proofing
   if(thread == nullptr) //If there's no thread, make one
   {
      threadFinished = false;
      thread = new std::thread(&SFDrive::PIDDrive, this, inches, maxVel, timeout, ZeroVelocityAtEnd);
      return true;
   }
   if(threadFinished) //If there is a thread but it's done, delete it and make another one
   {
      threadFinished = false;
      joinAutoThread();
      delete thread;
      thread = new std::thread(&SFDrive::PIDDrive, this, inches, maxVel, timeout, ZeroVelocityAtEnd);
      return true;
   }
   return false; //If thread already executing, do nothing
}

bool SFDrive::PIDTurn(float degreesClockwise, float radius, float maxVel, float timeout, bool ZeroVelocityAtEnd){

   int setPoint = 0, endPoint, innerChordLen;
   float endAngle = abs(degreesClockwise);
   int maxVelDelta = maxVel / m_wheelCircumference * m_ticksPerRev;
   double startTime, currStepTime, lastStepTime, deltaTime;

   //zero encoder
   disableP();
   setLeftMotorPosition(0);
   setRightMotorPosition(0);
   setLeftMotorSetpoint(0);
   setRightMotorSetpoint(0);
   enableP();

   if (radius < m_wheelTrack / 2)
      radius = m_wheelTrack / 2.0f;
   endPoint = (((endAngle / 360.0f) * ((radius + m_wheelTrack / 2.0f) * 2 * m_PI)) / m_wheelCircumference) * m_ticksPerRev;
   innerChordLen = (((endAngle / 360.0f) * ((radius - m_wheelTrack / 2.0f) * 2 * m_PI)) / m_wheelCircumference) * m_ticksPerRev;

   startTime = lastStepTime = Timer().GetFPGATimestamp();

   if (ZeroVelocityAtEnd)
   {
      startTime = lastStepTime = Timer().GetFPGATimestamp();
      while ((int) setPoint < (int) endPoint && startTime + timeout > lastStepTime)
      {
         if(stopThread)
         {
            stopThread = false;
            break;
         }
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
            setLeftMotorSetpoint(setPoint * -1);
            setRightMotorSetpoint(innerSet);
         }
         else
         {
            setLeftMotorSetpoint(innerSet * -1);
            setRightMotorSetpoint(setPoint);
         }
      }
   }
   else
   {
      while ((int) setPoint < (int) endPoint && startTime + timeout > lastStepTime)
      {
         if(stopThread)
         {
            stopThread = false;
            break;
         }
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
            setLeftMotorSetpoint(setPoint * -1);
            setRightMotorSetpoint(innerSet);
         }
         else
         {
            setLeftMotorSetpoint(innerSet * -1);
            setRightMotorSetpoint(setPoint);
         }
      }
   }

   threadFinished = true;

   if (lastStepTime > startTime + timeout) //simple error check, did we finish the motion before we ran out of time
   {
      return false;
   }
   else
   {
      return true;
   }
}

bool SFDrive::PIDTurnThread(float degreesClockwise, float radius, float maxVel, float timeout, bool ZeroVelocityAtEnd){
   stopThread = false; //Idiot proofing
   if(thread == nullptr) //If there's no thread, make one
   {
      thread = new std::thread(&SFDrive::PIDTurn, this, degreesClockwise, radius, maxVel, timeout, ZeroVelocityAtEnd);
      return true;
   }
   if(threadFinished) //If there is a thread but it's done, delete it and make another one
   {
      threadFinished = false;
      joinAutoThread();
      delete thread;
      thread = new std::thread(&SFDrive::PIDTurn, this, degreesClockwise, radius, maxVel, timeout, ZeroVelocityAtEnd);
      return true;
   }
   return false; //If thread already executing, do nothing
}

void SFDrive::initPID(){
   setP(m_P);
   setI(m_I);
   setD(m_D);
}

void SFDrive::enableP(){
   setP(m_P);
}

void SFDrive::disableP(){
   setP(0);
}

void SFDrive::setAccel(float accl){
   m_maxAccl = accl;
}

void SFDrive::stopAutoThread()
{
   stopThread = true;
   joinAutoThread();
   stopThread = false;
}

void SFDrive::joinAutoThread()
{
   if(thread == nullptr || !thread->joinable())
   {
      return;
   }
   thread->join();
}