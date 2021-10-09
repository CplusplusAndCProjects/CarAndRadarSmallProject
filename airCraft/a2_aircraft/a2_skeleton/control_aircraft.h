#ifndef CONTROL_AIRCRAFT_H
#define CONTROL_AIRCRAFT_H

#include "simulator.h"
#include <thread>
#include <vector>
#include <iostream>

using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::RangeBearingStamped;
using geometry_msgs::RangeVelocityStamped;
using namespace simulator;

class Control_aircraft
{
private:
    /* data */
public:
    ~Control_aircraft();

    /*For example purposes only, this thread attemps to get the friendly aircraft's
    (red triangle) pose every 2 seconds. It plots this pose on the simulation (blue triangle)
    via the testPose, and this pose stays on for 1 second, as per the
    'testPose()' documentation in the simualtor class.*/
    static void exampleThread(const std::shared_ptr<Simulator> & sim);


    /*
    This thread will simply get the current velocity and feed it back into
    controlled velocity, at the designated minimum time required (watchdog time) refer
    'controlFriendly()' documentation in the simualtor class.
    */
    static void controlThread(const std::shared_ptr<Simulator> & sim);
    void ControlAircraftExample(int argc, char *argv[]);
    void EstimatePositionOfTheBogie();
};
#endif 