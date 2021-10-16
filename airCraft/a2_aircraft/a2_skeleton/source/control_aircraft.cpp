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

  bogieSize = sim->rangeVelocityToBogiesFromBase().size();
  threads.push_back(sim->spawn());
  threads.push_back(std::thread(controlThread, sim,0));
  threads.push_back(std::thread(EstimateTimeToPositionOfTheBogie, sim,0));

  //Join threads and end!
  for(auto & t: threads){
    t.join();
  }
}


//For example purposes only, this thread attemps to get the friendly aircraft's
//(red triangle) pose every 2 seconds. It plots this pose on the simulation (blue triangle)
// via the testPose, and this pose stays on for 1 second, as per the
//'testPose()' documentation in the simualtor class.
vector<double> Control_aircraft::EstimateTimeToPositionOfTheBogie(const std::shared_ptr<Simulator> & sim,int timer){
  
  //vector<double> times ;
  //vector<vector<EdgeInfo> > adjacencyListForMultiNode; 
  std::vector<Pose> poses;
  int count_Timer =0;
  while (true)
  {
    Pose pose = sim->getFriendlyPose();
    vector<geometry_msgs::Point> points = EstimatePositionOfTheBogie(sim);
    RangeBearingStamped miniumBogiesRangeBearingStamped_temp; 
    Analysis analysis(points);
    //adjacencyListForMultiNode = analysis.exportGraph();
    miniumBogiesRangeVelocityStamped = GetMiniumRangeVeclocityStampedVec(sim,analysis);
    times = analysis.timeToImpact(pose);
    // if(distanceWithStation > Simulator::AIRSPACE_SIZE/4)
    // {
    //   miniumBogiesRangeBearingStamped = GetMiniumRangeBearingStamped(sim, analysis);
    //   std::this_thread::sleep_for(std::chrono::milliseconds(timstampe));
    //   continue;
    // }
    //std::cout << "timesize = " <<timesize<<", times.size() = "<< times.size()<<std::endl;
    miniumBogiesRangeBearingStamped_temp = GetMiniumRangeBearingStamped(sim, analysis);
    double disnceBogiesTpAircrast = GetDistanceOfFriendlyWithBiogies(sim, miniumBogiesRangeBearingStamped_temp);
    if(distanceWithStation > Simulator::AIRSPACE_SIZE/4)
    {
      miniumBogiesRangeBearingStamped = miniumBogiesRangeBearingStamped_temp;
      std::this_thread::sleep_for(std::chrono::milliseconds(timstampe));
      continue;
    }
    miniumBogiesRangeBearingStamped = Estimate_FutureMiniumRangeBearingStamped(miniumBogiesRangeBearingStamped_temp, sim);
    std::this_thread::sleep_for(std::chrono::milliseconds(timstampe));
    if(timer!=0)
      {
        count_Timer +=1;
        if (count_Timer >= timer)
        {
          //sim->stop();
          break;
        }
      
    }
  }
  return times;
}
std::vector<Point> Control_aircraft:: EstimatePositionOfTheBogie(const std::shared_ptr<Simulator> & sim){

    vector<geometry_msgs::Point> points; 
    Pose pose = sim->getFriendlyPose();
    vector<geometry_msgs::RangeBearingStamped> rangeBearingStampedVec = sim->rangeBearingToBogiesFromFriendly();
    vector<geometry_msgs::RangeVelocityStamped> rangeVelocityStamped = sim->rangeVelocityToBogiesFromBase();

    for (int i = 0; i < rangeBearingStampedVec.size(); i++)
    {
      geometry_msgs::Point point = local2Global(rangeBearingStampedVec.at(i), pose);
      points.push_back(point);
      /* code */
    }

  return points;
}

void Control_aircraft:: controlThread(const std::shared_ptr<Simulator> & sim, int timer) {
    double lin = 0; 
    double ang = 0;

    double Omega_max = Simulator::G_MAX * 9.81 / Simulator::V_TERM; 
    int angularVelocity = sim->getFriendlyAngularVelocity();
    bool result ;
    bool isAircraftReturn = false;
    int count_old = 0;
    int count_current =0;
    int count_Timer =0;
    while(true){
    distanceWithStation = GetDistanceOfFriendlyWithStation(sim);
    //std::cout << "distanceWithStation= " <<distanceWithStation<<" AIRSPACE_SIZE= "<< Simulator::AIRSPACE_SIZE<< std::endl;
    if(distanceWithStation < Simulator::AIRSPACE_SIZE/2)
    {
      isAircraftReturn = false;
      //std::cout << "isAircraftReturn is false" <<std::endl;
    }
    if(distanceWithStation >= Simulator::AIRSPACE_SIZE/2)
    {
      isAircraftReturn = true;
      //std::cout << "DistanceWithStation > AIRSPACE_SIZE/2" <<std::endl;
      lin =  Simulator::V_TERM;//sim->getFriendlyLinearVelocity();//miniumBogiesRangeVelocityStamped.velocity;
      ang = Omega_max; //2*M_PI - sim->getFriendlyAngularVelocity(); 
    }
    else
    {   
        lin = Simulator::V_TERM;
        if(miniumBogiesRangeBearingStamped.bearing>M_PI)
        {
          ang = -Omega_max;
          //std::cout << "ang = -Omega_max" <<std::endl;
        }
        else if(miniumBogiesRangeBearingStamped.bearing>M_PI/10 && miniumBogiesRangeBearingStamped.bearing<M_PI )
        {
          ang = Omega_max;
          //std::cout << "ang = Omega_max;" <<std::endl;
        }
        else if (miniumBogiesRangeBearingStamped.bearing > M_PI/50 && miniumBogiesRangeBearingStamped.bearing<M_PI/10)
        {
          lin = Simulator::V_MAX;
          ang = 0;        
        }

        else
        {
          lin = Simulator::V_MAX;
          ang = 0;
        }

    }

    result = sim->controlFriendly(lin, ang);
    std::this_thread::sleep_for(std::chrono::milliseconds(timstampe));
    if(timer!=0)
      {
        count_Timer +=1;
        if (count_Timer >= timer)
        {
          sim->stop();
          break;
        }
      
    }
  }
}

RangeBearingStamped Control_aircraft::GetMiniumRangeBearingStamped(const std::shared_ptr<Simulator> & sim,Analysis &analysis){
   
    RangeBearingStamped miniumBogiesRangeBearingStamped1;
    std::vector<RangeBearingStamped> rangeBearingStampedVec  = sim->rangeBearingToBogiesFromFriendly();
    //std::cout << "rangeBearingStampedVec size: " << rangeBearingStampedVec.size() <<std::endl;
    if(timesize != times.size())
    {
      timesize = times.size();
      orderOfMiniumbogies = GetMiniumOderTimers(times);

    }
    miniumBogiesRangeBearingStamped1 = rangeBearingStampedVec.at(orderOfMiniumbogies);
    //std::cout << "minium bearing rangeBearingStampedVec: " << miniumBogiesRangeBearingStamped1.bearing <<std::endl;
    return miniumBogiesRangeBearingStamped1;
}
RangeVelocityStamped Control_aircraft::GetMiniumRangeVeclocityStampedVec(const std::shared_ptr<Simulator> & sim,Analysis &analysis){

    RangeVelocityStamped miniumBogiesRangeVelocityStamped1;

    std::vector<RangeVelocityStamped> rangeVeclocityStampedVec = sim->rangeVelocityToBogiesFromBase();
    //std::cout << "rangeVeclocityStampedVec size: " << rangeVeclocityStampedVec.size() <<std::endl;
    if(timesize != times.size())
    {
      timesize = times.size();
      orderOfMiniumbogies = GetMiniumOderTimers(times);

    }
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
double Control_aircraft:: GetDistanceOfFriendlyWithBiogies(const std::shared_ptr<Simulator> & sim, RangeBearingStamped rangeBearingPogies){
    Point bogies = local2Global(rangeBearingPogies, sim->getFriendlyPose());
  
    double euclideanDistance = sqrt(pow((bogies.x - sim->getFriendlyPose().position.x),2) + pow((sim->getFriendlyPose().position.y -bogies.y),2));
    return euclideanDistance;
}

Pose Control_aircraft:: Estimate_FutureBogie_Poses(queue<Point> bogie_point, queue<double> timesToBogie, double step_time)
{
    //cout<<"Function Estimate_FutureBogie_Poses"<<endl;
    Pose bogie_position;
    Point front_pose = bogie_point.front();      //get the first pose from the container
    Point back_pose = bogie_point.back();   
    double front_time = timesToBogie.front();     //get 1st the time stamp
    double back_time = timesToBogie.back();   
    //cout<<"Step1"<<endl;

    double estimation_time = back_time + step_time;
    bogie_position.position.x = front_pose.x + (estimation_time - front_time)/(back_time - front_time)*(back_pose.x-front_pose.x);
    bogie_position.position.y = front_pose.y + (estimation_time - front_time)/(back_time - front_time)*(back_pose.y-front_pose.y);
    double bogie_position_yaw = atan2((back_pose.y-front_pose.y) , (back_pose.x-front_pose.x ));
    bogie_position.orientation = yawToQuaternion(bogie_position_yaw);
    return bogie_position;
}


RangeBearingStamped Control_aircraft::Estimate_FutureMiniumRangeBearingStamped(RangeBearingStamped miniumRangeBearingStamped,const std::shared_ptr<Simulator> & sim){
  //cout<<"Function Estimate_FutureMiniumRangeBearingStamped"<<endl;
  RangeBearingStamped estimate_FutureMiniumRangeBearingStamped;// = miniumRangeBearingStamped;
            //!< Container to store all timestamp
  Point miniumBogiePoint = local2Global(miniumRangeBearingStamped,sim-> getFriendlyPose());

  int size_buffer = 5;
  if(bogie_point.size()<size_buffer)
  {
    bogie_point.push(miniumBogiePoint);
  } 
  else{
    bogie_point.pop();
    bogie_point.push(miniumBogiePoint);
  }

  if(timesToBogie.size()<size_buffer)
  {
    timesToBogie.push(miniumRangeBearingStamped.timestamp);
  } 
  else{
    timesToBogie.pop();
    timesToBogie.push(miniumRangeBearingStamped.timestamp);
  }
  // if(bogie_point.size()<size_buffer)
  //   return estimate_FutureMiniumRangeBearingStamped;
  Pose estimate_FutureMiniumBogiePoint = Estimate_FutureBogie_Poses(bogie_point, timesToBogie, timstampe);

  estimate_FutureMiniumRangeBearingStamped = global2local(estimate_FutureMiniumBogiePoint.position, sim-> getFriendlyPose());
  queue<Point> bogie_point_empty;              //!< Container of all pre-future estimation pose
  queue<double> timesToBogie_empty; 
  if(bogieSize != times.size())
  {
    std::swap(bogie_point, bogie_point_empty);
    std::swap(timesToBogie, timesToBogie_empty);
    bogieSize = times.size();
    cout<<"Clear queue"<<endl;
  }

  return estimate_FutureMiniumRangeBearingStamped;
}
