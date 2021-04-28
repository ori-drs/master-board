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
	double iq_sat = 8.0;
	double iq_sat_for_initialization = 2.0;
	double freq = 0.6;
	double amplitude = 4*M_PI;
	int state = 0;
	bool flag_logging = true;  // enable (true) or disable (false) the logging
    double dx = 2.0;    // velocity for slow moving to desired init position
    int motor_i = 0;
    bool motor_at_zero_position[12];
    double pos_error = 0;
    double goal_position = 0;

	// trajectory tracking
	TrajectoryGenerator trajectoryGenerator;
	TrajectoryParameters trajectoryParameters[N_SLAVES_CONTROLED*N_MOTORS_PER_BOARD];  // The variable p1 is declared with 'Point'
    std::fstream calibrationFile;
    calibrationFile.open ("calibration.bin", std::ios_base::in);
    while (!calibrationFile.is_open());

	motor_i = HRHAA;
	trajectoryParameters[motor_i].active = 1;
	trajectoryParameters[motor_i].amplitude = 7.;
    trajectoryParameters[motor_i].frequencyStart = 0.5;
    trajectoryParameters[motor_i].frequencyInclination = 0.01;
	trajectoryParameters[motor_i].frequencyEnd = 1.0;
	trajectoryParameters[motor_i].phaseshift = 0;//M_PI/4;
	trajectoryParameters[motor_i].start_pos = 3*M_PI/16*GEAR_RATIO;


	motor_i = HRHFE;
	trajectoryParameters[motor_i].active = 1;
	trajectoryParameters[motor_i].amplitude = 6.0;
    trajectoryParameters[motor_i].frequencyStart = 0.2;
    trajectoryParameters[motor_i].frequencyInclination = 0.01;
	trajectoryParameters[motor_i].frequencyEnd = 0.8;
	trajectoryParameters[motor_i].phaseshift = 0;//M_PI/4;
	trajectoryParameters[motor_i].start_pos = -0*M_PI/16*GEAR_RATIO;

	motor_i = HRK;
	trajectoryParameters[motor_i].active = 1;
	trajectoryParameters[motor_i].amplitude = -15.0;
    trajectoryParameters[motor_i].frequencyStart = 0.8;
    trajectoryParameters[motor_i].frequencyInclination = 0.01;
	trajectoryParameters[motor_i].frequencyEnd = 1.2;
	trajectoryParameters[motor_i].phaseshift = 0;
	trajectoryParameters[motor_i].start_pos = 0;



	motor_i = HLHAA;
	trajectoryParameters[motor_i].active = 1;
	trajectoryParameters[motor_i].amplitude = 7.;
    trajectoryParameters[motor_i].frequencyStart = 0.5;
    trajectoryParameters[motor_i].frequencyInclination = 0.01;
	trajectoryParameters[motor_i].frequencyEnd = 1.0;
	trajectoryParameters[motor_i].phaseshift = 0;//M_PI/4;
	trajectoryParameters[motor_i].start_pos = 3*M_PI/16*GEAR_RATIO;

	motor_i = HLHFE;
	trajectoryParameters[motor_i].active = 1;
	trajectoryParameters[motor_i].amplitude = 6.0;
    trajectoryParameters[motor_i].frequencyStart = 0.2;
    trajectoryParameters[motor_i].frequencyInclination = 0.01;
	trajectoryParameters[motor_i].frequencyEnd = 0.8;
	trajectoryParameters[motor_i].phaseshift = 0;//M_PI/4;
	trajectoryParameters[motor_i].start_pos = -0*M_PI/16*GEAR_RATIO;

	motor_i = HLK;
	trajectoryParameters[motor_i].active = 1;
	trajectoryParameters[motor_i].amplitude = -15.0;
    trajectoryParameters[motor_i].frequencyStart = 0.8;
    trajectoryParameters[motor_i].frequencyInclination = 0.01;
	trajectoryParameters[motor_i].frequencyEnd = 1.2;
	trajectoryParameters[motor_i].phaseshift = 0;
	trajectoryParameters[motor_i].start_pos = 0;



	motor_i = FRHAA;
	trajectoryParameters[motor_i].active = 0;
	trajectoryParameters[motor_i].amplitude = 7.;
    trajectoryParameters[motor_i].frequencyStart = 0.5;
    trajectoryParameters[motor_i].frequencyInclination = 0.01;
	trajectoryParameters[motor_i].frequencyEnd = 1.0;
	trajectoryParameters[motor_i].phaseshift = 0;//M_PI/4;
	trajectoryParameters[motor_i].start_pos = 3*M_PI/16*GEAR_RATIO;

	motor_i = FRHFE;
	trajectoryParameters[motor_i].active = 0;
	trajectoryParameters[motor_i].amplitude = 6.0;
    trajectoryParameters[motor_i].frequencyStart = 0.2;
    trajectoryParameters[motor_i].frequencyInclination = 0.01;
	trajectoryParameters[motor_i].frequencyEnd = 0.8;
	trajectoryParameters[motor_i].phaseshift = 0;//M_PI/4;
	trajectoryParameters[motor_i].start_pos = -0*M_PI/16*GEAR_RATIO;

	motor_i = FRK;
	trajectoryParameters[motor_i].active = 0;
	trajectoryParameters[motor_i].amplitude = -15.0;
    trajectoryParameters[motor_i].frequencyStart = 0.8;
    trajectoryParameters[motor_i].frequencyInclination = 0.01;
	trajectoryParameters[motor_i].frequencyEnd = 1.2;
	trajectoryParameters[motor_i].phaseshift = 0;
	trajectoryParameters[motor_i].start_pos = 0;




	motor_i = FLHAA;
	trajectoryParameters[motor_i].active = 0;
	trajectoryParameters[motor_i].amplitude = 7.;
    trajectoryParameters[motor_i].frequencyStart = 0.5;
    trajectoryParameters[motor_i].frequencyInclination = 0.01;
	trajectoryParameters[motor_i].frequencyEnd = 1.0;
	trajectoryParameters[motor_i].phaseshift = 0;//M_PI/4;
	trajectoryParameters[motor_i].start_pos = 3*M_PI/16*GEAR_RATIO;

	motor_i = FLHFE;
	trajectoryParameters[motor_i].active = 0;
	trajectoryParameters[motor_i].amplitude = 6.0;
    trajectoryParameters[motor_i].frequencyStart = 0.2;
    trajectoryParameters[motor_i].frequencyInclination = 0.01;
	trajectoryParameters[motor_i].frequencyEnd = 0.8;
	trajectoryParameters[motor_i].phaseshift = 0;//M_PI/4;
	trajectoryParameters[motor_i].start_pos = -0*M_PI/16*GEAR_RATIO;

	motor_i = FLK;
	trajectoryParameters[motor_i].active = 0;
	trajectoryParameters[motor_i].amplitude = -15.0;
    trajectoryParameters[motor_i].frequencyStart = 0.8;
    trajectoryParameters[motor_i].frequencyInclination = 0.01;
	trajectoryParameters[motor_i].frequencyEnd = 1.2;
	trajectoryParameters[motor_i].phaseshift = 0;
	trajectoryParameters[motor_i].start_pos = 0;


	nice(-20); //give the process a high priority
	printf("-- Main --\n");
	MasterBoardInterface robot_if(argv[1]);
	robot_if.Init();

	Logger logger(N_SLAVES_CONTROLED*N_MOTORS_PER_BOARD);   // create logger for motors
	if(flag_logging) // if flag for logging is true
	{
		logger.createFiles();  // create logger file
		logger.initLogs();	 // write header in log file
	}

	//Initialisation, send the init commands
	for (int i = 0; i < N_SLAVES_CONTROLED; i++)
	{
		robot_if.motor_drivers[i].motor1->SetCurrentReference(0.);
		robot_if.motor_drivers[i].motor2->SetCurrentReference(0.);
		robot_if.motor_drivers[i].motor1->Enable();
		robot_if.motor_drivers[i].motor2->Enable();
		
		// Set the gains for the PD controller running on the cards.
		robot_if.motor_drivers[i].motor1->set_kp(kp);
		robot_if.motor_drivers[i].motor2->set_kp(kp);
		robot_if.motor_drivers[i].motor1->set_kd(kd);
		robot_if.motor_drivers[i].motor2->set_kd(kd);

		// Set the maximum current controlled by the card.
		robot_if.motor_drivers[i].motor1->set_current_sat(iq_sat_for_initialization);
		robot_if.motor_drivers[i].motor2->set_current_sat(iq_sat_for_initialization);
		
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

					robot_if.motors[i].SetCurrentReference(0.);
//					robot_if.motors[i].SetPositionReference(trajectoryParameters[i].zero_pos);
//					robot_if.motors[i].SetVelocityReference(0.);

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
                state = 4;
                for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
				{                    
					//if (robot_if.motors[i].IsEnabled())
					{
                        goal_position = trajectoryGenerator.sinePos(trajectoryParameters[i], 0);
                        // goal_position = trajectoryParameters[i].zero_pos;   
                        pos_error = goal_position - robot_if.motors[i].GetPosition();
                        printf("%f\t", pos_error);
						double ref;
                        if (abs(pos_error) > 0.1*dx && !motor_at_zero_position[i])
                        {
							// printf("test0\n");
                            state = 3;
							// printf("test1\n");
                            ref = trajectoryParameters[i].startupPosition + t*dx*sgn(pos_error);  // new position in direction to init_pos
							printf("test2\n");
							// printf("test3\n");
                        }
						else
						{
                            motor_at_zero_position[i] = 1;
                            ref = goal_position;
						}
						double v_ref = 0;
						robot_if.motors[i].SetCurrentReference(0.);
						robot_if.motors[i].SetPositionReference(ref);
						robot_if.motors[i].SetVelocityReference(v_ref);
					}
				}
				printf("\n");
                break;

            // ****************** play trajectory ******************
            case 4:
                t = 0;
				for (int i = 0; i < N_SLAVES_CONTROLED; i++)
				{
					// Set the maximum current controlled by the card.
					robot_if.motor_drivers[i].motor1->set_current_sat(iq_sat);
					robot_if.motor_drivers[i].motor2->set_current_sat(iq_sat);
				}

                state = 5;
                break;
			case 5:
				//closed loop, position
				if(flag_logging)
					logger.writeImuLog(t, robot_if);    // log imu data
					
				for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
				{
                    if(flag_logging)
                        logger.writeMotorLog(t,robot_if.motors[i],i);  // log motor status
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
						double ref = trajectoryGenerator.sinePos(trajectoryParameters[i], t);
						double v_ref = 0;  //trajectoryGenerator.sineVel(trajectoryParameters[i], t);
						robot_if.motors[i].SetCurrentReference(0.);
						robot_if.motors[i].SetPositionReference(ref);
						robot_if.motors[i].SetVelocityReference(v_ref);
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
