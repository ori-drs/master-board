
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * This script runs the static identification method
 * for the static identification method of one actuator
 * 
 * It is basically just a constant torque applied on one actuator
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <assert.h>
#include <unistd.h>
#include <chrono>
#include <math.h>
#include <stdio.h>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <string>

#include "master_board_sdk/master_board_interface.h"
#include "master_board_sdk/defines.h"
#include "master_board_sdk/logger.h"

int main()
{
	// int cpt = 0;
	// double current_desired = 0;
	// double dt = 0.001;
	// double t = 0;
	// double iq_sat = 4.0;
	// double init_pos[N_SLAVES * 2] = {0};
	// int state = 0;

	printf("-- Main --\n");
	printf("-- Playground --\n");


	// std::string fileName;
	// std::string fileNameBase = "logMotor_";
	// std::string fileNameEx = ".log";

	Logger loggerMotor[12];
	double t = 23.865;
	for (int i=0;i<1;i++)
	{
		// fileName = fileNameBase + std::to_string(i) + fileNameEx;
		loggerMotor[i].createFile("logMotor" + std::to_string(i) + ".log");
		loggerMotor[i].initMotorLog();
	}

	Logger loggerIMU;
	loggerIMU.createFile("logIMU.log");
	loggerIMU.initImuLog();

	while(1);


// 	std::ofstream* logfile = new std::ofstream[12]; // create an array of logfiles
// 	logfile[0] = std::ofstream("test0.log");

// // Write to the file
// 	logfile[0] << "timeframe; position motor; velocity motor; position reference motor; velocity reference motor; current reference motor\n";
// 	int timeframe = 0;
// 	double Position = 10.423;
// 	double Velocity = 19.5482;
// 	double PositionReference = 2.54;
// 	double VelocityReference = 1.974;
// 	logfile[0] << timeframe << ";";
// 	logfile[0] << Position << ";";
// 	logfile[0] << Velocity << ";";
// 	logfile[0] << PositionReference << ";";
// 	logfile[0] << VelocityReference << ";";
// 	logfile[0] << "\n";

    // log.createFile();
    // printf("test\n");

	return 0;
}
