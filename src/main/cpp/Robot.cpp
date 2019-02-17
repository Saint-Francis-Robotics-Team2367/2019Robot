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

void Robot::RobotInit() 
{
    
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
    arduino1 = new I2C(I2C::Port::kOnboard, 4);

}

void Robot::TeleopPeriodic() 
{
    //arduino1->Transaction(NULL,0,myTestInteger,1);
    //int myTemporaryTestInteger = *myTestInteger;
    //std::cout<<myTemporaryTestInteger<<std::endl;
    //uint8_t myTestInteger2 = 4;
	//arduino1->Transaction(&myTestInteger2, 1, NULL, 0);
    //arduino1->Transaction(NULL, 0, myTestInteger, 1);
    test->Set(0.0);
    //arduino1->Read(4, 32, ourData);
    //int myTestInteger;
    //myTestInteger <<= 8;
    //myTestInteger |= *ourData;
    //string myTestString = to_string(myTestInteger);
    //int myTemporaryTestInteger = *myTestInteger;
    //cout<<myTestString<<endl;

    
    //char data[] = new char[MAXBYTES];//create a char array to hold the incoming data
	bool readData = arduino1->Read(4, MAXBYTES, ourData);//use address 4 on i2c and store it in data
    if(readData){
        string myBirdie(reinterpret_cast< char const* >(ourData)); //some uint8_t operation to string
	    DriverStation::ReportError(myBirdie);
    }
    //string output(data);//create a string from the byte array
    
	//might not need these last two lines
    //int pt = output.indexOf((char)255);
	//return (String) output.subSequence(0, pt < 0 ? 0 : pt);//im not sure what these last two lines do
    
    
    //DriverStation::ReportError(myTestString);
}

void Robot::TestPeriodic() 
{
  
}

void Robot::DisabledInit(){

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif