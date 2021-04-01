/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
* Creates a trajectory on joint level
* 
* To use the trajectory Generator:
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include "master_board_sdk/trajectoryGenerator.h"

double TrajectoryGenerator::sinePos(double init_pos, TrajectoryParameters para, double t)
{
    return init_pos + para.amplitude * sin((2 * M_PI * para.frequency * t)+para.phaseshift);
}

double TrajectoryGenerator::sineVel(double init_pos, TrajectoryParameters para, double t)
{
    return 2. * M_PI * para.frequency * para.amplitude * cos((2 * M_PI * para.frequency * t)+para.phaseshift);
}
