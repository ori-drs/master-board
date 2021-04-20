#include <iostream>
#include <fstream>

struct TrajectoryParameters
{
    bool active = 0;
    double frequencyStart = 0.6;
    double frequencyInclination = 0.0;
    double frequencyEnd = 0.6;
    double amplitude = 0;
    double phaseshift = 0;  // in rad
    double init_pos = 0;    // value where the sine curve will cycle around
    double zero_pos = 0;    // starting value when robot is startet (should be zero normally because of encoder reset)
    double startupPosition = 0;    // starting value when robot is startet (should be zero normally because of encoder reset)
    double start_pos = 0;
};

class TrajectoryGenerator
{
public:
    double sinePos(TrajectoryParameters para, double t);
    double sineVel(TrajectoryParameters para, double t);
    double frequency(TrajectoryParameters para, double t);
};

