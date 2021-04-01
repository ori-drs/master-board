#include <iostream>
#include <fstream>

struct TrajectoryParameters
{
    bool active = 0;
    double frequency = 0.6;
    double amplitude = 0;
    double phaseshift = 0;  // in rad
};

class TrajectoryGenerator
{
public:
    double sinePos(double init_pos, TrajectoryParameters para, double t);
    double sineVel(double init_pos, TrajectoryParameters para, double t);
};

