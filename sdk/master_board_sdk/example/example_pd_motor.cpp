#include <assert.h>
#include <unistd.h>
#include <chrono>
#include <math.h>
#include <stdio.h>
#include <sys/stat.h>

#include "master_board_sdk/master_board_interface.h"
#include "master_board_sdk/defines.h"
#include "master_board_sdk/logger.h"
#include "master_board_sdk/trajectoryGenerator.h"

#define N_SLAVES_CONTROLED 6
#define N_MOTORS_PER_BOARD 2

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        throw std::runtime_error("Please provide the interface name "
                                 "(i.e. using 'ifconfig' on linux");
    }

	int cpt = 0;
	double dt = 0.001;
	double t = 0;
	double kp = 5.;
	double kd = 0.1;
	double iq_sat = 1.5;
	double freq = 0.6;
	double amplitude = 4*M_PI;
	double init_pos[N_SLAVES * 2] = {0};
	int state = 0;
	bool flag_logging = false;  // enable (true) or disable (false) the logging

	// trajectory tracking
	TrajectoryGenerator trajectoryGenerator;
	struct TrajectoryParameters
	{
		bool active = 0;
		double freq = 0.6;
		double amplitude = 3;
	} trajectoryParameters[N_SLAVES_CONTROLED*N_MOTORS_PER_BOARD];  // The variable p1 is declared with 'Point'

	int motor_i = 0;


	motor_i = 0;  // HRK
	trajectoryParameters[motor_i].active = 1;
	trajectoryParameters[motor_i].amplitude = 10;

	motor_i = 1;  // HLK
	trajectoryParameters[motor_i].active = 1;
	trajectoryParameters[motor_i].amplitude = -10;

	motor_i = 2;  // HRHFE
	trajectoryParameters[motor_i].active = 1;
	trajectoryParameters[motor_i].amplitude = -3;

	motor_i = 3;  // HLHFE
	trajectoryParameters[motor_i].active = 1;
	trajectoryParameters[motor_i].amplitude = 3;

	motor_i = 4;  // HRHAA
	trajectoryParameters[motor_i].active = 1;
	trajectoryParameters[motor_i].amplitude = -3;

	motor_i = 5;  // HLHAA
	trajectoryParameters[motor_i].active = 1;
	trajectoryParameters[motor_i].amplitude = -3;


	motor_i = 6;  // FRHFE
	trajectoryParameters[motor_i].active = 1;
	trajectoryParameters[motor_i].amplitude = 3;

	motor_i = 7;  // FLHFE
	trajectoryParameters[motor_i].active = 1;
	trajectoryParameters[motor_i].amplitude = -3;

	motor_i = 8;  // FRHFE
	trajectoryParameters[motor_i].active = 1;
	trajectoryParameters[motor_i].amplitude = 10;

	motor_i = 9;  // FLHFE
	trajectoryParameters[motor_i].active = 1;
	trajectoryParameters[motor_i].amplitude = -10;

	motor_i = 10;  // FRHAA
	trajectoryParameters[motor_i].active = 1;
	trajectoryParameters[motor_i].amplitude = -3;

	motor_i = 11;  // FLHAA
	trajectoryParameters[motor_i].active = 1;
	trajectoryParameters[motor_i].amplitude = -3;


	nice(-20); //give the process a high priority
	printf("-- Main --\n");
	MasterBoardInterface robot_if(argv[1]);
	robot_if.Init();

	Logger loggerIMU;	// create logger for IMU
	Logger loggerMotor[N_SLAVES_CONTROLED*N_MOTORS_PER_BOARD];   // create logger for motors
	if(flag_logging) // if flag for logging is true
	{
		loggerIMU.createFile("example_IMU.log");  // create logger file
		loggerIMU.initImuLog();	 // write header in log file
		for (int i=0; i<N_SLAVES_CONTROLED*N_MOTORS_PER_BOARD; i++)  // create files for motor logging
		{
			loggerMotor[i].createFile("example_Motor" + std::to_string(i) + ".log");
			loggerMotor[i].initMotorLog();
		}
	}

	//Initialisation, send the init commands
	for (int i = 0; i < N_SLAVES_CONTROLED; i++)
	{
		robot_if.motor_drivers[i].motor1->SetCurrentReference(0);
		robot_if.motor_drivers[i].motor2->SetCurrentReference(0);
		robot_if.motor_drivers[i].motor1->Enable();
		robot_if.motor_drivers[i].motor2->Enable();
		
		// Set the gains for the PD controller running on the cards.
		robot_if.motor_drivers[i].motor1->set_kp(kp);
		robot_if.motor_drivers[i].motor2->set_kp(kp);
		robot_if.motor_drivers[i].motor1->set_kd(kd);
		robot_if.motor_drivers[i].motor2->set_kd(kd);

		// Set the maximum current controlled by the card.
		robot_if.motor_drivers[i].motor1->set_current_sat(iq_sat);
		robot_if.motor_drivers[i].motor2->set_current_sat(iq_sat);
		
		robot_if.motor_drivers[i].EnablePositionRolloverError();
		robot_if.motor_drivers[i].SetTimeout(5);
		robot_if.motor_drivers[i].Enable();
	}

	std::chrono::time_point<std::chrono::system_clock> last = std::chrono::system_clock::now();
	while (!robot_if.IsTimeout() && !robot_if.IsAckMsgReceived()) {
		if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last)).count() > dt)
		{
			last = std::chrono::system_clock::now();
			robot_if.SendInit();
		}
	}

	if (robot_if.IsTimeout())
	{
		printf("Timeout while waiting for ack.\n");
	}

	while (!robot_if.IsTimeout())
	{
		if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last)).count() > dt)
		{
			last = std::chrono::system_clock::now(); //last+dt would be better
			cpt++;
			t += dt;
			robot_if.ParseSensorData(); // This will read the last incomming packet and update all sensor fields.
			switch (state)
			{
			case 0: //check the end of calibration (are the all controlled motor enabled and ready?)
				state = 1;
				for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
				{
					if (!robot_if.motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave

					if (!(robot_if.motors[i].IsEnabled() && robot_if.motors[i].IsReady()))
					{
						state = 0;
					}
					init_pos[i] = robot_if.motors[i].GetPosition(); //initial position

					// Use the current state as target for the PD controller.
					robot_if.motors[i].SetCurrentReference(0.);
					robot_if.motors[i].SetPositionReference(init_pos[i]);
					robot_if.motors[i].SetVelocityReference(0.);

					t = 0;	//to start sin at 0
				}
				break;
			case 1:
				//closed loop, position
				if(flag_logging)
					loggerIMU.writeImuLog(t, robot_if);    // log imu data
					
				for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
				{
					if (i % 2 == 0)
					{
						if (!robot_if.motor_drivers[i/2].is_connected) 
						{
							//printf("motor driver %d is not connected\n", i/2); // ignoring the motors of a disconnected slave
							continue; 
						}

						// making sure that the transaction with the corresponding µdriver board succeeded
						if (robot_if.motor_drivers[i / 2].error_code == 0xf)
						{
							// printf("Transaction with SPI%d failed\n", i / 2);
							continue; //user should decide what to do in that case, here we ignore that motor
						}
					}
					if (robot_if.motors[i].IsEnabled() && trajectoryParameters[i].active)
					{

						double ref = trajectoryGenerator.sinePos(init_pos[i], trajectoryParameters[i].amplitude, trajectoryParameters[i].freq, t);
						double v_ref = trajectoryGenerator.sineVel(init_pos[i], trajectoryParameters[i].amplitude, trajectoryParameters[i].freq, t);
						robot_if.motors[i].SetCurrentReference(0.);
						robot_if.motors[i].SetPositionReference(ref);
						robot_if.motors[i].SetVelocityReference(v_ref);
						if(flag_logging)
							loggerMotor[i].writeMotorLog(t,robot_if.motors[i]);  // log motor status
					}
				}
				break;
			}
			
			if (cpt % 100 == 0)
			{
				printf("\33[H\33[2J"); //clear screen
				robot_if.PrintIMU();
				robot_if.PrintADC();
				robot_if.PrintMotors();
				robot_if.PrintMotorDrivers();
				robot_if.PrintStats();
				fflush(stdout);
				

			}
			robot_if.SendCommand(); //This will send the command packet
		}
		else
		{
			std::this_thread::yield();
		}
	}
        printf("Masterboard timeout detected. Either the masterboard has been shut down or there has been a connection issue with the cable/wifi.\n");
	return 0;
}
