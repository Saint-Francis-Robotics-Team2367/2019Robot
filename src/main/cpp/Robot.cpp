/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

//union TransferBytes transferBytes;
//int counter = 0;

void Robot::RobotInit() 
{
    //highByte = 0;
    //lowByte = 0;
}

void Robot::RobotPeriodic() 
{
    
}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() 
{

    arduino1 = new SerialPort(9600, SerialPort::Port::kUSB1, 8, SerialPort::kParity_None, SerialPort::kStopBits_One);
    arduino1->SetReadBufferSize(8);
    //arduino1->Read(ourData,32);
}

void Robot::TeleopPeriodic() 
{

    char testChar = ' ';
    char * otherTestChar = &testChar;
    arduino1->Read(otherTestChar,1);

    float recvd = (float) *otherTestChar;
    recvd = recvd / 2.0;


    if(recvd == 0 && !firstSensor){
        firstSensor = true;
    }
    else if(firstSensor){
        if(recvd == 1 || recvd==0) DriverStation::ReportError("Sensor 1 is defected!");
        std::cout<<recvd<<std::endl;
        firstSensorValue = recvd;
        firstSensor = false;
    } else {
        if(recvd == 1 || recvd==0) DriverStation::ReportError("Sensor 2 is defected!");
        secondSensorValue = recvd;
std::cout<<recvd<<std::endl;
    }

    

    
    arduino1->SetTimeout(0.5);
    //might have to setup a timeout to give it time to read
    arduino1->EnableTermination('\n');
    std::cout<<recvd<<std::endl;
    //string data = to_string(recvd);
    //DriverStation::ReportError(data);
    //I don't know why I still get the watchdog error :(

}



void Robot::TestPeriodic() 
{
  
}

void Robot::DisabledInit(){

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif