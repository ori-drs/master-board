#include <iostream>
#include <fstream>
#include "master_board_sdk/defines.h"
#include "master_board_sdk/motor.h"
#include "master_board_sdk/master_board_interface.h"


class Motor;
class Logger
{
public:
  void createFile(const std::string& fileName);
  void initMotorLog();
  void writeMotorLog(double timestamp, Motor motor);
  void initImuLog();
  void writeImuLog(double timestamp, MasterBoardInterface robot_if);
  void closeFile();

private:
  std::ofstream Logfile;
};
