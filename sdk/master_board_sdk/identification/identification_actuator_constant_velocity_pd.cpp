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

    bool transition = true;
	int cpt = 0;
	double dt = 0.001;
	double t = 0;
    double t_transition = 0;
	double kp = 5.;
	double kd = 0.1;
	double iq_sat = 6.0;
	double velocity = 220;
    double speedup = 70;
	double init_pos[N_SLAVES * 2] = {0};
	double init_pos_b[N_SLAVES * 2] = {0};
	int state = 0;
	bool flag_logging = true;  // enable (true) or disable (false) the logging
	double maxVelocity = 400;   // maximum allowed velocity before emergency stop

	nice(-20); //give the process a high priority
	printf("-- Main --\n");
	MasterBoardInterface robot_if(argv[1]);
	robot_if.Init();

	Logger loggerIMU;	// create logger for IMU
	Logger loggerMotor[N_SLAVES_CONTROLED*N_MOTORS_PER_BOARD];   // create logger for motors
	if(flag_logging) // if flag for logging is true
	{
		loggerIMU.createFile("identification_actuator_constant_velocity_IMU.log");  // create logger file
		loggerIMU.initImuLog();	 // write header in log file
		for (int i=0; i<N_SLAVES_CONTROLED*N_MOTORS_PER_BOARD; i++)  // create files for motor logging
		{
			loggerMotor[i].createFile("identification_actuator_constant_velocity_motor" + std::to_string(i) + ".log");
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
				if(flag_logging)
					loggerIMU.writeImuLog(t, robot_if);    // log imu data
				for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
				{
					if (i % 2 == 0)
					{
						if (!robot_if.motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave
						if (robot_if.motor_drivers[i / 2].error_code == 0xf) continue; //user should decide what to do in that case, here we ignore that motor
					}

   					if(robot_if.motors[i].GetVelocity() > maxVelocity) // if motor's velocity exceed the maximum allowed velocity
						state = 3;  // go to emergency stop

					if (robot_if.motors[i].IsEnabled())
					{
                        double ref = init_pos[i] + 0.5*speedup*pow(t,2);
                        double v_ref = speedup*t;

                        if (v_ref>velocity)
                        {
                            state = 2;
                            init_pos[i] = ref;
                            t_transition = t;
                        }

						robot_if.motors[i].SetCurrentReference(0.);
						robot_if.motors[i].SetPositionReference(ref);
						robot_if.motors[i].SetVelocityReference(v_ref);
						if(flag_logging)
							loggerMotor[i].writeMotorLog(t,robot_if.motors[i]);  // log motor status
					}
				}
				break;
			case 2:
				if(flag_logging)
					loggerIMU.writeImuLog(t, robot_if);    // log imu data
				for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
				{
					if (i % 2 == 0)
					{
						if (!robot_if.motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave
						if (robot_if.motor_drivers[i / 2].error_code == 0xf) continue; //user should decide what to do in that case, here we ignore that motor
					}

   					if(robot_if.motors[i].GetVelocity() > maxVelocity) // if motor's velocity exceed the maximum allowed velocity
						state = 3;  // go to emergency stop

					if (robot_if.motors[i].IsEnabled())
					{
                        double ref = init_pos[i] + velocity*(t-t_transition);
                        double v_ref = velocity;

						robot_if.motors[i].SetCurrentReference(0.);
						robot_if.motors[i].SetPositionReference(ref);
						robot_if.motors[i].SetVelocityReference(v_ref);
						if(flag_logging)
							loggerMotor[i].writeMotorLog(t,robot_if.motors[i]);  // log motor status
					}
				}
				break;
            case 3:
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
