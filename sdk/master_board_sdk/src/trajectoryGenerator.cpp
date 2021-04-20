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

double TrajectoryGenerator::sinePos(TrajectoryParameters para, double t)
{
    if (para.active)
        return para.zero_pos + para.start_pos + para.amplitude * sin((2 * M_PI * this->frequency(para, t) * t) + para.phaseshift);
    else
        return para.zero_pos;
}

double TrajectoryGenerator::sineVel(TrajectoryParameters para, double t)
{
    if (para.active)
        return 2. * M_PI * this->frequency(para, t) * para.amplitude * cos((2 * M_PI * this->frequency(para, t) * t) + para.phaseshift);
    else
        return 0;
}

double TrajectoryGenerator::frequency(TrajectoryParameters para, double t)
{
    double frequency_ = para.frequencyStart + para.frequencyInclination*t;
    if (frequency_ <= para.frequencyEnd)
        frequency_ = para.frequencyEnd;
    return frequency_;
}