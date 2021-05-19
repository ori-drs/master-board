/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
* Creates a csv file with using semicolon as delimiter
* to separate the data fields. Every line is one time step.
* The first line contains the description of the colums.
* Make sure that the folder
* 
* To use the logger:
* 	Logger logger(int NoMotors);     // create logger for motors and IMU
* 	logger.createFiles();            // create files for the logger
* 	logger.initLogs();               // write headline in file
*   logger.writeMotorLog(timestamp, motor, motorNr)   // writes the log for motorNr
*   logger.writeImuLog(timestamp, &robot_if)   // writes the log for the IMU
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include "master_board_sdk/defines.h"
#include "master_board_sdk/logger.h"
#include "master_board_sdk/motor.h"
#include "master_board_sdk/master_board_interface.h"

Logger::Logger(int nMotors)
{
    this->_nMotors = nMotors;   // number of motors
}

void Logger::createFiles()
{
    /* create files for each motor (12 for solo12) and for the IMU */
    // get date and time as string
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer,sizeof(buffer),"%y%m%d_%H%M%S",timeinfo);
    std::string str(buffer);

    // provide strings to construct the file name
    std::string logpath = "log/";           // folder
    std::string fileNameMotor = "Motor";    // name of file
    std::string fileNameIMU = "IMU";        // name of file
    std::string motorNr;
    std::string date = str;
    std::string extension = ".log";
    std::string fileNameMotorFull;
    std::string fileNameIMUFull;
    for(int i=0; i<this->_nMotors; i++)
    {
        motorNr = std::to_string(i);
        fileNameMotorFull = logpath + date + "_" + fileNameMotor + motorNr + extension;
        LogfileMotors[i] = std::ofstream(fileNameMotorFull);  // create file
        if(!LogfileMotors[i].is_open()) {
            printf("failed to create a motor logfile. Does the folder exist (should be log/)?\n");
            while(1);
        }
    }    
    fileNameIMUFull = logpath + date + "_" + fileNameIMU + extension;
    LogfileIMU = std::ofstream(fileNameIMUFull);  // create file
    if(!LogfileIMU.is_open()) {
        printf("failed to create IMU logfile. Does the folder exist (should be log/)?");
        while(1);
    }
}

void Logger::initLogs()
{
    // Writes a list of the column content in the file's header
    for(int i=0; i<this->_nMotors; i++)
    {
        this->LogfileMotors[i] << "timestamp [s]" << ";";
        this->LogfileMotors[i] << "position [rad]" << ";";
        this->LogfileMotors[i] << "velocity [rad/s]" << ";";
        this->LogfileMotors[i] << "current [A]" << ";";
        this->LogfileMotors[i] << "position offset [rad]" << ";";
        this->LogfileMotors[i] << "position reference [rad]" << ";";
        this->LogfileMotors[i] << "velocity reference [rad/s]" << ";";
        this->LogfileMotors[i] << "current reference [A]" << ";";
        this->LogfileMotors[i] << "current saturation [A]" << ";";
        this->LogfileMotors[i] << "kp [A/rad]" << ";";
        this->LogfileMotors[i] << "kd [As/rad]";
        this->LogfileMotors[i] << "\n";
        this->LogfileMotors[i].flush();  // write to file
    }

        this->LogfileIMU << "timestamp" << ";";
        this->LogfileIMU << "accelerometer x" << ";";
        this->LogfileIMU << "accelerometer y" << ";";
        this->LogfileIMU << "accelerometer z" << ";";
        this->LogfileIMU << "gyroscope x" << ";";
        this->LogfileIMU << "gyroscope y" << ";";
        this->LogfileIMU << "gyroscope z" << ";";
        this->LogfileIMU << "attitude roll" << ";";
        this->LogfileIMU << "attitude pitch" << ";";
        this->LogfileIMU << "attitude yaw" << ";";
        this->LogfileIMU << "linear acceleration x" << ";";
        this->LogfileIMU << "linear acceleration y" << ";";
        this->LogfileIMU << "linear acceleration z";
        this->LogfileIMU << "\n";  // end of measurement line
        this->LogfileIMU.flush();  // write to file
}

void Logger::writeMotorLog(double timestamp, Motor motor, int motorNr)
{
    // stores the motor measurements and reference values into log file    
    int i = motorNr;
    this->LogfileMotors[i] << timestamp << ";";
    this->LogfileMotors[i] << motor.GetPosition() << ";";
    this->LogfileMotors[i] << motor.GetVelocity() << ";";
    this->LogfileMotors[i] << motor.GetCurrent() << ";";
    this->LogfileMotors[i] << motor.GetPositionOffset() << ";";
    this->LogfileMotors[i] << motor.position_ref << ";";  // [rad]
    this->LogfileMotors[i] << motor.velocity_ref << ";";  // [rad/s]
    this->LogfileMotors[i] << motor.current_ref << ";";   // [A]
    this->LogfileMotors[i] << motor.current_sat << ";";   // [A]
    this->LogfileMotors[i] << motor.kp << ";";            // [A/rad]
    this->LogfileMotors[i] << motor.kd;            // [As/rad]
    this->LogfileMotors[i] << "\n";  // end of measurement line
    this->LogfileMotors[i].flush();  // write to file
}

void Logger::writeImuLog(double timestamp, MasterBoardInterface *robot_if)
{
    // stores the IMU measurements into log file
    int i = 0;
    imu_data_t imu_data = robot_if->GetIMU();
    this->LogfileIMU << timestamp << ";";
    this->LogfileIMU << imu_data.accelerometer[0] << ";";
    this->LogfileIMU << imu_data.accelerometer[1] << ";";
    this->LogfileIMU << imu_data.accelerometer[2] << ";";
    this->LogfileIMU << imu_data.gyroscope[0] << ";";
    this->LogfileIMU << imu_data.gyroscope[1] << ";";
    this->LogfileIMU << imu_data.gyroscope[2] << ";";
    this->LogfileIMU << imu_data.attitude[0] << ";";
    this->LogfileIMU << imu_data.attitude[1] << ";";
    this->LogfileIMU << imu_data.attitude[2] << ";";
    this->LogfileIMU << imu_data.linear_acceleration[0] << ";";
    this->LogfileIMU << imu_data.linear_acceleration[1] << ";";
    this->LogfileIMU << imu_data.linear_acceleration[2];
    this->LogfileIMU << "\n";  // end of measurement line
    this->LogfileIMU.flush();  // write to file
}

void Logger::closeFiles()
{
    for(int i=0; i<this->_nMotors; i++)
    {
        this->LogfileMotors[i].close();
    }
    this->LogfileIMU.close();
    printf("file closed\n");
}