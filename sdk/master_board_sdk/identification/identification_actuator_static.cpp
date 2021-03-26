
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * This script runs the dynamic identification method
 * for the dynamic identification method of one actuator
 * 
 * It applies a sine curve on the torque output of the actuator
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <assert.h>
#include <unistd.h>
#include <chrono>
#include <math.h>
#include <stdio.h>
#include <sys/stat.h>

#include "master_board_sdk/master_board_interface.h"
#include "master_board_sdk/defines.h"
#include "master_board_sdk/logger.h"

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
	double iq_sat_pos = 4.0;   // upper current limit
	double iq_sat_neg = -7.0;  // lower current limit, motor max 15 Amps
	double freq = 0.05;
	double amplitude_start = -0.50;
	double amplitude_end = -7.00;
	double amplitude_step = -0.25;
	double amplitude = amplitude_start;
	int period_num = 0;
	int state = 0;
	double maxVelocity = 30;   // maximum allowed velocity before emergency stop

	nice(-20); //give the process a high priority
	printf("-- Main --\n");
	MasterBoardInterface robot_if(argv[1]);
	// Logger log("identification_actuator_sine");  // create logger object
	robot_if.Init();
	//Initialisation, send the init commands
	for (int i = 0; i < N_SLAVES_CONTROLED; i++)
	{
		robot_if.motor_drivers[i].motor1->SetCurrentReference(0);
		robot_if.motor_drivers[i].motor2->SetCurrentReference(0);
		robot_if.motor_drivers[i].motor1->Enable();
		robot_if.motor_drivers[i].motor2->Enable();
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
				for (int i = 0; i < N_SLAVES_CONTROLED * N_MOTORS_PER_BOARD; i++)
				{
					if (!robot_if.motor_drivers[i / N_MOTORS_PER_BOARD].is_connected) continue; // ignoring the motors of a disconnected slave

					if (!(robot_if.motors[i].IsEnabled() && robot_if.motors[i].IsReady()))
					{
						state = 0;
					}
					t = 0;	//to start sin at 0
				}
				break;
			case 1:
				//open loop, current
				for (int i = 0; i < N_SLAVES_CONTROLED * N_MOTORS_PER_BOARD; i++)
				{
					if (i % 2 == 0)
					{
						if (!robot_if.motor_drivers[i / N_MOTORS_PER_BOARD].is_connected) continue; // ignoring the motors of a disconnected slave

						// making sure that the transaction with the corresponding µdriver board succeeded
						if (robot_if.motor_drivers[i / 2].error_code == 0xf)
						{
							printf("Transaction with SPI%d failed\n", i / N_MOTORS_PER_BOARD);
							continue; //user should decide what to do in that case, here we ignore that motor
						}
					}

					if(robot_if.motors[i].GetVelocity() > maxVelocity) // if motors too fast
						state = 2;  // emergency stop


					if (robot_if.motors[i].IsEnabled())
					{
						period_num = t*(freq*2);
						amplitude = amplitude_start + period_num*amplitude_step;
						double cur = amplitude * abs(sin(2 * M_PI * freq * t));   // M_PI is pi from math package

						if (cur > iq_sat_pos) {
							cur = iq_sat_pos;
							printf("current limit reached\n");
						}
						if (cur < iq_sat_neg) {
							cur = iq_sat_neg;
							printf("current limit reached\n");
						}
						robot_if.motors[i].SetCurrentReference(cur);
					}
				}
				break;
			case 2:
				printf("rotating too fast -> emergency stop\n");
				for (int i = 0; i < N_SLAVES_CONTROLED; i++)
				{
					robot_if.motors[i].SetCurrentReference(0);
					robot_if.motors[i*2].SetCurrentReference(0);
					robot_if.motor_drivers[i].motor1->Disable();
					robot_if.motor_drivers[i].motor2->Disable();
				}
				while(1);
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
				printf("period: %d\n", period_num);
				printf("amplitude: %.2f A\n", amplitude);
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
