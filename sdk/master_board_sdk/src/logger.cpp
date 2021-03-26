/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
* Creates a csv file with using semicolon as delimiter
* to separate the data fields. Every line is one time step.
* The first line contains the description of the colums.
* 
* To use the logger:
* 	Logger logIMU;                      // create logger for IMU
* 	logIMU.createFile("logIMU.log");    // create file for the logger
* 	logIMU.initImuLog();                // write headline in file for IMU
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include "master_board_sdk/defines.h"
#include "master_board_sdk/logger.h"
#include "master_board_sdk/motor.h"
#include "master_board_sdk/master_board_interface.h"

void Logger::createFile(const std::string& fileName)
{
    // ensures uniqueness of filename (add number on filename if already exists) and create file
    std::string logpath = "log/";  // stores in folder
    int name_idx = 0;   // index if file already exists
    std::string fileNameApp = logpath + fileName;
    std::ifstream file_to_check (fileNameApp.c_str());  // try to open file to check if it exists
    while (file_to_check.is_open())
    {
        name_idx++;  // increase index for filename
        file_to_check.close();
        fileNameApp = logpath + std::to_string(name_idx) + fileName;  // add index to filename
        file_to_check.open(fileNameApp.c_str());  // try to open file to check if it exists
    }
    Logfile = std::ofstream(fileNameApp);  // create file
    if(!Logfile.is_open()) 
        printf("failed to create logfile");
}

void Logger::initMotorLog()
{
    // Writes a list of the column content in the file's header
    this->Logfile << "timestamp [s]" << ";";
    this->Logfile << "position [rad]" << ";";
    this->Logfile << "velocity [rad/s]" << ";";
    this->Logfile << "current [A]" << ";";
    this->Logfile << "position offset [rad]" << ";";
    this->Logfile << "position reference [rad]" << ";";
    this->Logfile << "velocity reference [rad/s]" << ";";
    this->Logfile << "current reference [A]" << ";";
    this->Logfile << "current saturation [A]" << ";";
    this->Logfile << "kp [A/rad]" << ";";
    this->Logfile << "kd [As/rad]";
    this->Logfile << "\n";
    this->Logfile.flush();  // write to file
}

void Logger::writeMotorLog(double timestamp, Motor motor)
{
    // stores the motor measurements and reference values into log file    
    this->Logfile << timestamp << ";";
    this->Logfile << motor.GetPosition() << ";";
    this->Logfile << motor.GetVelocity() << ";";
    this->Logfile << motor.GetCurrent() << ";";
    this->Logfile << motor.GetPositionOffset() << ";";
    this->Logfile << motor.position_ref << ";";  // [rad]
    this->Logfile << motor.velocity_ref << ";";  // [rad/s]
    this->Logfile << motor.current_ref << ";";   // [A]
    this->Logfile << motor.current_sat << ";";   // [A]
    this->Logfile << motor.kp << ";";            // [A/rad]
    this->Logfile << motor.kd;            // [As/rad]
    this->Logfile << "\n";  // end of measurement line
    this->Logfile.flush();  // write to file
}

void Logger::initImuLog()
{
    // Writes a list of the column content in the file's header
    this->Logfile << "timestamp" << ";";
    this->Logfile << "accelerometer x" << ";";
    this->Logfile << "accelerometer y" << ";";
    this->Logfile << "accelerometer z" << ";";
    this->Logfile << "gyroscope x" << ";";
    this->Logfile << "gyroscope y" << ";";
    this->Logfile << "gyroscope z" << ";";
    this->Logfile << "attitude roll" << ";";
    this->Logfile << "attitude pitch" << ";";
    this->Logfile << "attitude yaw" << ";";
    this->Logfile << "linear acceleration x" << ";";
    this->Logfile << "linear acceleration y" << ";";
    this->Logfile << "linear acceleration z";
    this->Logfile << "\n";  // end of measurement line
    this->Logfile.flush();  // write to file
}


void Logger::writeImuLog(double timestamp, MasterBoardInterface robot_if)
{
    // stores the IMU measurements into log file
    imu_data_t imu_data = robot_if.GetIMU();
    this->Logfile << timestamp << ";";
    this->Logfile << imu_data.accelerometer[0] << ";";
    this->Logfile << imu_data.accelerometer[1] << ";";
    this->Logfile << imu_data.accelerometer[2] << ";";
    this->Logfile << imu_data.gyroscope[0] << ";";
    this->Logfile << imu_data.gyroscope[1] << ";";
    this->Logfile << imu_data.gyroscope[2] << ";";
    this->Logfile << imu_data.attitude[0] << ";";
    this->Logfile << imu_data.attitude[1] << ";";
    this->Logfile << imu_data.attitude[2] << ";";
    this->Logfile << imu_data.linear_acceleration[0] << ";";
    this->Logfile << imu_data.linear_acceleration[1] << ";";
    this->Logfile << imu_data.linear_acceleration[2];
    this->Logfile << "\n";  // end of measurement line
    this->Logfile.flush();  // write to file
}

void Logger::closeFile()
{
    this->Logfile.close();
    printf("file closed\n");
}