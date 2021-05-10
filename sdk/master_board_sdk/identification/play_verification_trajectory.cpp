/* this program moves every joint individual a little bit in a direction. This
offers the possibility to see if the logging matches to the real movements. Just import
the log files into the evaluation script and see if the movements in its replay fits
to the movements the robot does in real. Espacially for the directions of the joints */ 

#include <assert.h>
#include <unistd.h>
#include <chrono>
#include <math.h>
#include <stdio.h>
#include <sys/stat.h>
#include <curses.h>
#include <string.h>

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
	double iq_sat = 4.0;
	double iq_sat_for_initialization = 2.0;
	double freq = 0.6;
	double amplitude = 4*M_PI;
	int state = 0;
	bool flag_logging = true;  // enable (true) or disable (false) the logging
    double dx = 2.0;    // velocity for slow moving to desired init position
    int motor_i = 0;
    double pos_error = 0;
    double goal_position = 0;
    int activeMotor = 0;
    bool motor_at_zero_position[12];
    double pos_start[12];
    double current[12];
	double q_ref[12];
	double dq_ref[12];
    char delimiter;
	bool CURRENT_CONTOL = true;   // false for pd control, true for current feedforward

	// trajectory tracking
	TrajectoryGenerator trajectoryGenerator;
	TrajectoryParameters trajectoryParameters[N_SLAVES_CONTROLED*N_MOTORS_PER_BOARD];  // The variable p1 is declared with 'Point'

    std::fstream trajectoryFile_current;
    // trajectoryFile_current.open("ValidationTrajectoryURDFCurrent_FL.out", std::ios_base::in);
    trajectoryFile_current.open("ValTraj_Current_FL.out", std::ios_base::in);
    while (!trajectoryFile_current.is_open());

    std::fstream trajectoryFile_q;
    // trajectoryFile_q.open("ValidationTrajectoryURDFQ_FL.out", std::ios_base::in);
    trajectoryFile_q.open("ValTraj_Q_FL.out", std::ios_base::in);
    while (!trajectoryFile_q.is_open());

    std::fstream trajectoryFile_dq;
    // trajectoryFile_dq.open("ValidationTrajectoryURDFDQ_FL.out", std::ios_base::in);
    trajectoryFile_dq.open("ValTraj_DQ_FL.out", std::ios_base::in);
    while (!trajectoryFile_dq.is_open());

    std::fstream trajectoryFile_ddq;
    // trajectoryFile_dq.open("ValidationTrajectoryURDFDDQ_FL.out", std::ios_base::in);
    trajectoryFile_ddq.open("ValTraj_DDQ_FL.out", std::ios_base::in);
    while (!trajectoryFile_ddq.is_open());

    std::fstream calibrationFile;
    calibrationFile.open ("calibration.bin", std::ios_base::in);
    while (!calibrationFile.is_open());






	nice(-20); //give the process a high priority
	printf("-- Main --\n");
	MasterBoardInterface robot_if(argv[1]);
	robot_if.Init();

	Logger logger(N_SLAVES_CONTROLED*N_MOTORS_PER_BOARD);   // create logger for motors
	if(flag_logging)    // if flag for logging is true
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


            // ****************** drive slowly to start position ******************
            case 2:
                t = 0;
                pos_start[0]=0; pos_start[1]=0; pos_start[2]=0; pos_start[3]=0; pos_start[4]=0;  pos_start[5]=0; 
                pos_start[6]=0; pos_start[7]=0; pos_start[8]=0; pos_start[9]=0; pos_start[10]=0; pos_start[11]=0; 
                trajectoryFile_q >> pos_start[FLHAA] >> delimiter >> pos_start[FLHFE] >> delimiter >> pos_start[FLK];
                state = 3;
                break;
            case 3:
                state = 4;
                for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
				{
					//if (robot_if.motors[i].IsEnabled())
					{
                        goal_position = trajectoryParameters[i].zero_pos + pos_start[i];
                        pos_error = goal_position - robot_if.motors[i].GetPosition();
                        // printf("%f\t", pos_error);
                        double ref = 0;
                        double vref = 0;
                        if (abs(pos_error) > 0.1*dx && !motor_at_zero_position[i])
                        {
                            state = 3;
                            ref = trajectoryParameters[i].startupPosition + t*dx*sgn(pos_error);  // new position in direction to init_pos
                            vref = 0;
                        }
						else
						{
                            motor_at_zero_position[i] = 1;
                            ref = goal_position;
                            vref = 0;
						}
                        robot_if.motors[i].SetCurrentReference(0.);
                        robot_if.motors[i].SetPositionReference(ref);
                        robot_if.motors[i].SetVelocityReference(vref);
					}
				}
				// printf("\n");
                break;

            // ****************** play trajectory ******************
            case 4:
                t = 0;
                state = 5;
				if(CURRENT_CONTOL)
				{
					kp = 0;
					kd = 0;
				}
				for (int i = 0; i < N_SLAVES_CONTROLED; i++)
				{
					// Set the maximum current controlled by the card.
					robot_if.motor_drivers[i].motor1->set_current_sat(iq_sat);
					robot_if.motor_drivers[i].motor2->set_current_sat(iq_sat);
                    robot_if.motor_drivers[i].motor1->set_kp(kp);
                    robot_if.motor_drivers[i].motor2->set_kp(kp);
                    robot_if.motor_drivers[i].motor1->set_kd(kd);
                    robot_if.motor_drivers[i].motor2->set_kd(kd);
				}
                break;
			case 5:
				// printf("play trajecotry\n");
                if (t>3.960/3) //(t > 3.960/7)
                {
                    t=0;
                    trajectoryFile_q.clear();
                    trajectoryFile_q.seekg(0);
                    trajectoryFile_dq.clear();
                    trajectoryFile_dq.seekg(0);
                    trajectoryFile_ddq.clear();
                    trajectoryFile_ddq.seekg(0);
                    trajectoryFile_current.clear();
                    trajectoryFile_current.seekg(0);
                }
				memset(current, 0, sizeof(current)); // for automatically-allocated arrays
				memset(q_ref, 0, sizeof(q_ref)); // for automatically-allocated arrays
				memset(dq_ref, 0, sizeof(dq_ref)); // for automatically-allocated arrays
                // current[0] = 0; current[1] = 0; current[2] = 0; current[3] = 0; current[4] = 0;  current[5] = 0; 
                // current[6] = 0; current[7] = 0; current[8] = 0; current[9] = 0; current[10] = 0; current[11] = 0;
                trajectoryFile_current >> current[FLHAA] >> delimiter >> current[FLHFE] >> delimiter >> current[FLK];  // read one line
                trajectoryFile_q >> q_ref[FLHAA] >> delimiter >> q_ref[FLHFE] >> delimiter >> q_ref[FLK];
				trajectoryFile_dq >> dq_ref[FLHAA] >> delimiter >> dq_ref[FLHFE] >> delimiter >> dq_ref[FLK];
				
                // printf("curent: %f\t%f\t%f\n", current[FLHAA], current[FLHFE], current[FLK]);
				// printf("q_ref: %f\t%f\t%f\n", q_ref[FLHAA], q_ref[FLHFE], q_ref[FLK]);
				// printf("dq_ref: %f\t%f\t%f\n", dq_ref[FLHAA], dq_ref[FLHFE], dq_ref[FLK]);
				// printf("\n");

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
					if (robot_if.motors[i].IsEnabled())
					{
                        double ref = trajectoryParameters[i].zero_pos + q_ref[i];
						double v_ref = dq_ref[i];
                        double cur = current[i];

						if (CURRENT_CONTOL)
						{
							robot_if.motors[i].SetCurrentReference(cur);
						}
						else
						{
							robot_if.motors[i].SetPositionReference(ref);
							robot_if.motors[i].SetVelocityReference(v_ref);
						}
					}
				}
				break;
            case 7:
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
