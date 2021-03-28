#include <iostream>
#include <fstream>

class TrajectoryGenerator
{
public:
    double sinePos(double init_pos, double amplitude, double frequency, double t);
    double sineVel(double init_pos, double amplitude, double frequency, double t);
};

