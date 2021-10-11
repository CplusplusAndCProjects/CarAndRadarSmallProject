#include "control_aircraft.h"
#include"tf2.h"
Control_aircraft::~Control_aircraft()
{
}


void Control_aircraft::ControlAircraft(int argc, char *argv[]){
           GameMode game_mode = GameMode::BASIC;
   //If code is started with --advanced, it will run in advanced mode
   //std::cout << "Run with: " << argv[0] << " --advanced to run in advanced mode" << std::endl;
   if(argc>1){
       if(strcmp (argv[1],"-advanced")){
           std::cout << "Advanced Mode Activated" << std::endl;
           game_mode = GameMode::ADVANCED;
       }
   }

  std::vector<std::thread> threads;

  //Create a shared pointer for the simulator class
  std::shared_ptr<Simulator> sim(new Simulator(true,game_mode));

  //Showing how to get airspace size (for example), constants arer accessible
  //std::cout << "Airspace Size:" << Simulator::AIRSPACE_SIZE << std::endl;


  threads.push_back(sim->spawn());
  threads.push_back(std::thread(controlThread, sim));
  threads.push_back(std::thread(exampleThread, sim));
  threads.push_back(std::thread(EstimateTimeToPositionOfTheBogie, sim));

  //Join threads and end!
  for(auto & t: threads){
    t.join();
  }
}


//For example purposes only, this thread attemps to get the friendly aircraft's
//(red triangle) pose every 2 seconds. It plots this pose on the simulation (blue triangle)
// via the testPose, and this pose stays on for 1 second, as per the
//'testPose()' documentation in the simualtor class.
vector<double> Control_aircraft::EstimateTimeToPositionOfTheBogie(const std::shared_ptr<Simulator> & sim){
  
  //vector<double> times ;
  vector<vector<EdgeInfo> > adjacencyListForMultiNode; 
  std::vector<Pose> poses;
  while (true)
  {
    Pose pose = sim->getFriendlyPose();
    vector<geometry_msgs::Point> points = EstimatePositionOfTheBogie(sim);
    Analysis analysis(points);
    adjacencyListForMultiNode = analysis.exportGraph();
    times = analysis.timeToImpact(pose);
    miniumBogiesRangeVelocityStamped = GetMiniumRangeVeclocityStampedVec(sim,analysis);
    miniumBogiesRangeBearingStamped = GetMiniumRangeBearingStamped(sim, analysis);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return times;
}
std::vector<Point> Control_aircraft:: EstimatePositionOfTheBogie(const std::shared_ptr<Simulator> & sim){

    vector<geometry_msgs::Point> points; 
    Pose pose = sim->getFriendlyPose();
    vector<geometry_msgs::RangeBearingStamped> rangeBearingStampedVec = sim->rangeBearingToBogiesFromFriendly();

    for (int i = 0; i < rangeBearingStampedVec.size(); i++)
    {
      geometry_msgs::Point point = local2Global(rangeBearingStampedVec.at(i), pose);
      points.push_back(point);
      /* code */
    }

  return points;
}

void Control_aircraft:: exampleThread(const std::shared_ptr<Simulator> & sim) {
  int count = 0;
  while(true) {
    //Get the friendly aircraft's position and orientation
    Pose pose = sim->getFriendlyPose();
    pose.orientation = tf::yawToQuaternion(M_PI/4);
    std::vector<Pose> poses;
    poses.push_back(pose);
    sim->testPose(poses);

   std::vector<RangeVelocityStamped> rangeVeclocityStampedVec = sim->rangeVelocityToBogiesFromBase();
   std::vector<RangeBearingStamped> rangeBearingStampedVec = sim->rangeBearingToBogiesFromFriendly();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

  }
}

void Control_aircraft:: controlThread(const std::shared_ptr<Simulator> & sim) {
    double lin = 0; 
    double ang = 0;
    double Omega_max = Simulator::G_MAX * 9.81 / Simulator::V_TERM; 
    int angularVelocity = sim->getFriendlyAngularVelocity();
    int distanceWithStation = 0;  
    bool result ;
    bool isAircraftReturn = false;
    int count_old = 0;
    int count_current =0;
    while(true){
    
    distanceWithStation = GetDistanceOfFriendlyWithStation(sim);
    //std::cout << "distanceWithStation= " <<distanceWithStation<<" AIRSPACE_SIZE= "<< Simulator::AIRSPACE_SIZE<< std::endl;
    if(distanceWithStation < Simulator::AIRSPACE_SIZE/2)
    {
      isAircraftReturn = false;
      //std::cout << "isAircraftReturn is false" <<std::endl;
    }
    if(distanceWithStation >= Simulator::AIRSPACE_SIZE/2 && isAircraftReturn == false)
    {
      isAircraftReturn = true;
      //std::cout << "DistanceWithStation > AIRSPACE_SIZE/2" <<std::endl;
      lin =  Simulator::V_TERM;//sim->getFriendlyLinearVelocity();//miniumBogiesRangeVelocityStamped.velocity;
      ang = Omega_max; //2*M_PI - sim->getFriendlyAngularVelocity(); 
    }

    else
    {
        //std::cout << "bogies stoped! " << std::endl;
        lin = Simulator::V_TERM;
        if(miniumBogiesRangeBearingStamped.bearing>M_PI)
        {
          ang = -Omega_max;
        }
        else if(miniumBogiesRangeBearingStamped.bearing>M_PI/25 && miniumBogiesRangeBearingStamped.bearing<M_PI )
        {
          ang = Omega_max;
        }
        else
        {
          lin = Simulator::V_MAX;
          ang = 0;
        }

    }

    // if(count_current != count_old)
    // {
    //   result = sim->controlFriendly(lin, ang);
    //   std::this_thread::sleep_for(std::chrono::milliseconds(50));
    //   count_old = count_current;
    // }
    //std::cout << "lin= " << lin<<", ang= "<<ang <<std::endl;
    //std::cout << "result set: " << result <<std::endl;
    result = sim->controlFriendly(lin, ang);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

RangeBearingStamped Control_aircraft::GetMiniumRangeBearingStamped(const std::shared_ptr<Simulator> & sim,Analysis &analysis){
   
    RangeBearingStamped miniumBogiesRangeBearingStamped1;
    std::vector<RangeBearingStamped> rangeBearingStampedVec  = sim->rangeBearingToBogiesFromFriendly();
    //std::cout << "rangeBearingStampedVec size: " << rangeBearingStampedVec.size() <<std::endl;
    int orderOfMiniumbogies = GetMiniumOderTimers(times);
    miniumBogiesRangeBearingStamped1 = rangeBearingStampedVec.at(orderOfMiniumbogies);
    //std::cout << "minium bearing rangeBearingStampedVec: " << miniumBogiesRangeBearingStamped1.bearing <<std::endl;
    return miniumBogiesRangeBearingStamped1;
}
RangeVelocityStamped Control_aircraft::GetMiniumRangeVeclocityStampedVec(const std::shared_ptr<Simulator> & sim,Analysis &analysis){

    RangeVelocityStamped miniumBogiesRangeVelocityStamped1;
    std::vector<RangeVelocityStamped> rangeVeclocityStampedVec = sim->rangeVelocityToBogiesFromBase();
    //std::cout << "rangeVeclocityStampedVec size: " << rangeVeclocityStampedVec.size() <<std::endl;
    int orderOfMiniumbogies = GetMiniumOderTimers(times);
    miniumBogiesRangeVelocityStamped1 = rangeVeclocityStampedVec.at(orderOfMiniumbogies);
    //std::cout << "minium velocity rangeBearingStampedVec: " << miniumBogiesRangeVelocityStamped1.velocity <<std::endl;
    return miniumBogiesRangeVelocityStamped1;

}

double Control_aircraft::GetMiniumOderTimers(vector<double> times){


    double miniumTime = 0, orderOfMiniumbogies = 0;
    //std::cout << "times size: " << times.size() <<std::endl;
    for (int i = 0; i < times.size(); i++)
    {
        if(miniumTime> times.at(i))
          miniumTime = times.at(i);
    }
    for (int i = 0; i < times.size(); i++)
    {
        if(miniumTime = times.at(i))
          orderOfMiniumbogies = i;
          break;
    }
    return orderOfMiniumbogies;
}
double Control_aircraft:: GetDistanceOfFriendlyWithStation(const std::shared_ptr<Simulator> & sim){

    double euclideanDistance = sqrt(pow((Simulator::BSTATION_LOC.x - sim->getFriendlyPose().position.x),2) + pow((sim->getFriendlyPose().position.y - Simulator::BSTATION_LOC.y),2));
    return euclideanDistance;

}
double GetDistanceOfFriendlyWithBiogies(const std::shared_ptr<Simulator> & sim){
  
}


