#ifndef CONTROL_AIRCRAFT_H
#define CONTROL_AIRCRAFT_H

#include "simulator.h"
#include <thread>
#include <vector>
#include <iostream>
#include"tf2.h"
#include"tf.h"
#include"analysis.h"
using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::RangeBearingStamped;
using geometry_msgs::RangeVelocityStamped;
using namespace simulator;
using namespace std;
using namespace tf2;
using namespace tf;

static vector<double> times ;
static RangeVelocityStamped miniumBogiesRangeVelocityStamped;// = GetMiniumRangeVeclocityStampedVec(sim);

static RangeBearingStamped miniumBogiesRangeBearingStamped;// = GetMiniumRangeBearingStamped(sim);

static queue<Point> bogie_point;              //!< Container of all pre-future estimation pose
static queue<double> timesToBogie; 
static int bogieSize=0, timesize=0;
static const int timstampe = 30;
static int orderOfMiniumbogies =0; 
static int distanceWithStation = 0;  
class Control_aircraft
{
private:
    /* data */
    Simulator sim1;
    Pose miniumbogies;
public:
    ~Control_aircraft();
    /*
    This thread will simply get the current velocity and feed it back into
    controlled velocity, at the designated minimum time required (watchdog time) refer
    'controlFriendly()' documentation in the simualtor class.
    */
    static void controlThread(const std::shared_ptr<Simulator> & sim,int timer);
    void ControlAircraft(int argc, char *argv[]);
    static vector<double> EstimateTimeToPositionOfTheBogie(const std::shared_ptr<Simulator> & sim,int timer);
    static std::vector<Point> EstimatePositionOfTheBogie( const std::shared_ptr<Simulator> & sim);
    static RangeBearingStamped GetMiniumRangeBearingStamped(const std::shared_ptr<Simulator> & sim, Analysis &analysis);
    static RangeVelocityStamped GetMiniumRangeVeclocityStampedVec(const std::shared_ptr<Simulator> & sim, Analysis &analysis);
    static double GetMiniumOderTimers(vector<double> times);
    static double GetDistanceOfFriendlyWithStation(const std::shared_ptr<Simulator> & sim);
    static double GetDistanceOfFriendlyWithBiogies(const std::shared_ptr<Simulator> & sim, RangeBearingStamped rangeBearingPogies);
    static Pose Estimate_FutureBogie_Poses(queue<Point> bogie_point, queue<double> timesToBogie, double step_time);
    static RangeBearingStamped Estimate_FutureMiniumRangeBearingStamped(RangeBearingStamped miniumRangeBearingStamped,const std::shared_ptr<Simulator> & sim);
};
#endif