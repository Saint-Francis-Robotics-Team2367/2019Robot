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
#include <frc/I2C.h>
#include <frc/DriverStation.h>
#include <frc/Spark.h>
#include <frc/SerialPort.h>

using namespace frc;
using namespace std;

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
  const int MAXBYTES = 32;  
  Spark *test = new Spark(1);


 private:
    char * ourData;
    SerialPort * arduino1;


    bool firstSensor;
    int firstSensorChecker;
    int secondSensorChecker;

    int firstSensorValue;
    int secondSensorValue;

};