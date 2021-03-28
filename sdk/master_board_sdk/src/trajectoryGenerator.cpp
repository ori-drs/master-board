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

double TrajectoryGenerator::sinePos(double init_pos, double amplitude, double frequency, double t)
{
    return init_pos + amplitude * sin(2 * M_PI * frequency * t);
}

double TrajectoryGenerator::sineVel(double init_pos, double amplitude, double frequency, double t)
{
    return 2. * M_PI * frequency * amplitude * cos(2 * M_PI * frequency * t);
}
