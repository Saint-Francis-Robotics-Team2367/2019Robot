/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <ctre/Phoenix.h>
#include "SFDrive_SparkMax.h"
#include <frc/Joystick.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/DriverStation.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Servo.h>
#include <SmartSender.h>
#include <frc/Timer.h>
#include <MotionProfile.h>
#include <Ultra.h>

using namespace frc;

class Robot : public frc::TimedRobot {
   public:
      void RobotInit() override;
      void RobotPeriodic() override;
      void AutonomousInit() override;
      void AutonomousPeriodic() override;
      void TeleopInit() override;
      void TeleopPeriodic() override;
      void TestPeriodic() override;
      void DisabledInit() override;
      void ConfigPIDS();
      void placeHatchThreaded();
      //Control variables
      //bool operatorInCargoMode = true; //false = hatch panel mode
      //bool rumbleDriver = false;
      //bool rumbleOperator = false;
      //bool outputtingCargo = false;
      //double outputtingCargoStartTime = 0;
      //double elevatorGranularControlMultiplier = 320;
      //int hatchMechState = 0;
      //bool hatchMechStateSwitched = true;
      //int setpoint = 0;
      //bool elevatorFlag = false;
      //int hatchMechSetpoint = 0;
      bool autonOverride = false; //KILL THE AI REVOLUTION
      int autonState = 0;
      int maxVel = 60;
      /*double topServoUpSetpoint = 0;
      double topServoDownSetpoint = 0;
      double bottomServoUpSetpoint = 0;
      double bottomServoDownSetpoint = 67;
        */
      //Motor IDs
      const int rMotorFrontNum = 13;
      const int rMotorBackNum = 12;
      const int lMotorFrontNum = 16;
      const int lMotorBackNum = 3;
     // const int elevatorMotorNum = 1;
      //const int cargoIntakeMotorNum = 15;
      //const int cargoLeftMotorNum = 10;
      //const int cargoRightMotorNum = 14;
      //const int cargoTopMotorNum = 12;

      //Solenoid IDs 
      
      //const int cargoMechLeftSolenoidNum = 4;
      //const int cargoMechRightSolenoidNum = 2;
      // const int hatchMechSolenoidNum = 0;

      //Servo IDs
      //const int hatchMechTopServoNum = 2;
      //const int hatchMechBottomServoNum = 4;

      //Drive Constants
      const double pConstantDrive = 1;
      const double iConstantDrive = 0;
      const double dConstantDrive = 0;
      const double fConstantDrive = 0;
      const int driveMotorCurrentLimit = 40;
      // Auton Constants
      /*const double distanceToCorrection = 68.235;
      const double hatchPanelCentralOffset = 10.88;
      const double startingAngle = std::atan(hatchPanelCentralOffset/distanceToCorrection)*(180/M_PI); // arctan of the offset distance of the hatch panel and the distance to correction start - converted to radians
      const double startingDistance = std::sqrt(std::pow(hatchPanelCentralOffset, 2) + std::pow(distanceToCorrection, 2)); // the hypotenuse of that triangle
      const double correctionSensitivity = 40; // ticks per joystick unit
      const double correctionPeriodVelocity = 6519; // ticks per second (30in/s for safety * 217.3 ticks/in)
      const double reverseDistance = -20; // inches */
      // Auton non-consts
      //double xCorrect;
      //double rSetpointCorrect = 0;
      //double lSetpointCorrect = 0;
      //std::thread *hatchThread;
      //Elevator Constants
      /*const int cargoRocket1 = -10135;
      const int cargoRocket2 = -24271;
      const int cargoRocket3 = -37080; 
      const int cargoShip = -15366;
      const int hatchRocket2 = -14565;
      const int hatchRocket3 = -30711;
      const double pConstantElevator = 1;
      const double iConstantElevator = 0;
      const double dConstantElevator = 0;
      const int elevatorPeakMotorCurrentLimit = 40;
      const int elevatorContinuousMotorCurrentLimit = 30;
      const int elevatorPeakMotorCurrentLimitDuration = 500;//in milliseconds
      */

      //Drive motors
      rev::CANSparkMax * lMotorFront = new rev::CANSparkMax(lMotorFrontNum, rev::CANSparkMax::MotorType::kBrushless);
      rev::CANSparkMax * lMotorBack = new rev::CANSparkMax(lMotorBackNum, rev::CANSparkMax::MotorType::kBrushless);
      rev::CANSparkMax * rMotorBack = new rev::CANSparkMax(rMotorBackNum, rev::CANSparkMax::MotorType::kBrushless);
      rev::CANSparkMax * rMotorFront = new rev::CANSparkMax(rMotorFrontNum, rev::CANSparkMax::MotorType::kBrushless);
      const int tickPerRev = 42;
      const double gearboxRatio = 10.71;
      const double inchesPerRev = 3.1415*6;
      //Manipulator Motors
      /*WPI_TalonSRX * elevatorMotor = new WPI_TalonSRX(elevatorMotorNum);
      WPI_TalonSRX * cargoIntakeMotor = new WPI_TalonSRX(cargoIntakeMotorNum);
      WPI_VictorSPX * cargoLeftMotor = new WPI_VictorSPX(cargoLeftMotorNum);
      WPI_VictorSPX * cargoRightMotor = new WPI_VictorSPX(cargoRightMotorNum);
      WPI_VictorSPX * cargoTopMotor = new WPI_VictorSPX(cargoTopMotorNum); */

      //Solenoids
     /* DoubleSolenoid * cargoMechLeftSolenoid = new DoubleSolenoid(cargoMechLeftSolenoidNum, cargoMechLeftSolenoidNum + 1);
      DoubleSolenoid * cargoMechRightSolenoid = new DoubleSolenoid(cargoMechRightSolenoidNum, cargoMechRightSolenoidNum + 1);
      DoubleSolenoid * hatchMechSolenoid = new DoubleSolenoid(hatchMechSolenoidNum, hatchMechSolenoidNum + 1); */

      //Servo
      /*Servo * hatchMechTopServo = new Servo(hatchMechTopServoNum);
      Servo * hatchMechBottomServo = new Servo(hatchMechBottomServoNum); */

      //SfDrive Object
      SFDrive_SparkMax * myRobot = new SFDrive_SparkMax(lMotorFront, rMotorFront, pConstantDrive, iConstantDrive, dConstantDrive, fConstantDrive);
      
      //SmartSender Object
      // SmartSender * sender = new SmartSender();

      //Joysticks
      Joystick * driverStick = new Joystick(0);
      Joystick * operatorStick = new Joystick(1);
    // Threading
    frc::Timer* motionTimer;
    Ultra *ultra = new Ultra(0, 1, 2, 3);
    double leftDist;
    double rightDist;
    double angle;
    bool moveFlag = false;
    motionProfiler *profile;
    const double profileAccel = 60; // inches per second
    const double profileSpeed = 60; // inches per second
    const double robotWidth = 25; // inches
    double arcLength;

};
