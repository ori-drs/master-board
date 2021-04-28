#include <iostream>
#include <fstream>
#include "master_board_sdk/defines.h"
#include "master_board_sdk/motor.h"
#include "master_board_sdk/master_board_interface.h"


class Motor;
class Logger
{
public:
  Logger(int nMotors);
  void createFiles();
  void initLogs();
  void writeMotorLog(double timestamp, Motor motor, int motorNr);
  void writeImuLog(double timestamp, MasterBoardInterface robot_if);
  void closeFiles();

private:
  int _nMotors;
  std::ofstream LogfileMotors[24];
  std::ofstream LogfileIMU;
};
