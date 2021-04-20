#include <assert.h>
#include <unistd.h>
#include <chrono>
#include <math.h>
#include <stdio.h>
#include <sys/stat.h>
 #include <curses.h>

#include "master_board_sdk/master_board_interface.h"
#include "master_board_sdk/defines.h"
#include "master_board_sdk/logger.h"
#include "master_board_sdk/trajectoryGenerator.h"

#define N_SLAVES_CONTROLED 6
#define N_MOTORS_PER_BOARD 2
#define GEAR_RATIO  9

#define HRK     0
#define HLK     1
#define HRHFE   2
#define HLHFE   3
#define HRHAA   4
#define HLHAA   5
#define FRHFE   6
#define FLHFE   7
#define FRK     8
#define FLK     9
#define FRHAA   10
#define FLHAA   11

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

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
	double iq_sat = 2.0;
	double freq = 0.6;
	double amplitude = 4*M_PI;
	int state = 0;
	bool flag_logging = true;  // enable (true) or disable (false) the logging
    double dx = 2.0;    // velocity for slow moving to desired init position
    int motor_i = 0;
    double pos_error = 0;
    double goal_position = 0;

	// trajectory tracking
	TrajectoryGenerator trajectoryGenerator;
	TrajectoryParameters trajectoryParameters[N_SLAVES_CONTROLED*N_MOTORS_PER_BOARD];  // The variable p1 is declared with 'Point'
    std::fstream calibrationFile;
    calibrationFile.open ("calibration.bin", std::ios_base::in);
    while (!calibrationFile.is_open());



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
		robot_if.motor_drivers[i].motor1->SetCurrentReference(0.);
		robot_if.motor_drivers[i].motor2->SetCurrentReference(0.);
		robot_if.motor_drivers[i].motor1->Enable();
		robot_if.motor_drivers[i].motor2->Enable();


		robot_if.motor_drivers[i].motor1->SetPositionReference(robot_if.motors[i].GetPosition());
		robot_if.motor_drivers[i].motor2->SetPositionReference(robot_if.motors[i].GetPosition());

		
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


	for (int i = 0; i < N_SLAVES_CONTROLED * N_MOTORS_PER_BOARD; i++)
	{
		calibrationFile >> trajectoryParameters[i].zero_pos;
		printf("%f\n", trajectoryParameters[i].zero_pos);
	}
	printf("reading calibiration file finished\n");


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
            // ******************************** wait until all motors are ready ************************
			case 0: //check the end of calibration (are the all controlled motor enabled and ready?)
				state = 1;
				for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
				{
					if (!robot_if.motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave

					if (!(robot_if.motors[i].IsEnabled() && robot_if.motors[i].IsReady()))
					{
						state = 0;
					}
					trajectoryParameters[i].startupPosition = robot_if.motors[i].GetPosition(); //initial position at startup
//
//					// Use the current state as target for the PD controller.
					robot_if.motors[i].SetCurrentReference(0.);

					t = 0;	//to start sin at 0
				}
				break;
            
            // ******************* read position to set values for zero_pos *******************************
            case 1:
                state = 2;
                t = 0;
				break;


            // ****************** drive slowly to zero position ******************
            case 2:
                t = 0;
                state = 3;
                break;
            case 3:
                state = 3;
                for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
				{                    
					//if (robot_if.motors[i].IsEnabled())
					{
                        goal_position = trajectoryParameters[i].zero_pos;   // trajectoryParameters[i].zero_pos + trajectoryParameters[i].start_pos
						// printf("goal position: %f\n", goal_position);
                        pos_error = goal_position - robot_if.motors[i].GetPosition();
						// printf("pos error at i=%d and with error %f\n", i, pos_error);
                        printf("%f\t", pos_error);
                        if (abs(pos_error) > 0.1*dx)
                        {
							// printf("test0\n");
                            state = 3;
							// printf("test1\n");
                            double ref = trajectoryParameters[i].startupPosition + t*dx*sgn(pos_error);  // new position in direction to init_pos
							printf("test2\n");
                            double v_ref = 0;
							// printf("test3\n");
                            robot_if.motors[i].SetCurrentReference(0.);
                            robot_if.motors[i].SetPositionReference(ref);
                            robot_if.motors[i].SetVelocityReference(v_ref);
                        }
						else
						{
                            robot_if.motors[i].SetCurrentReference(0.);
                            robot_if.motors[i].SetPositionReference(trajectoryParameters[i].zero_pos);
                            robot_if.motors[i].SetVelocityReference(0.);
						}
					}
				}
				printf("\n");
                break;

            // ****************** play trajectory ******************
            case 4:
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
