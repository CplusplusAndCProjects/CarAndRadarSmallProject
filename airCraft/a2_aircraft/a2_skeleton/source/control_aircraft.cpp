#include "control_aircraft.h"
#include"tf2.h"
Control_aircraft::~Control_aircraft()
{
}
//For example purposes only, this thread attemps to get the friendly aircraft's
//(red triangle) pose every 2 seconds. It plots this pose on the simulation (blue triangle)
// via the testPose, and this pose stays on for 1 second, as per the
//'testPose()' documentation in the simualtor class.
vector<double> Control_aircraft::EstimateTimeToPositionOfTheBogie(const std::shared_ptr<Simulator> & sim){
  
  vector<double> times ;
  std::vector<Pose> poses;
  while (true)
  {
    Pose pose = sim->getFriendlyPose();

    vector<geometry_msgs::Point> points = EstimatePositionOfTheBogie(sim);
    Analysis analysis(points);
    times = analysis.timeToImpact(pose);
    
    for (int i = 0; i < points.size(); i++)
    {

      std::cout << "Time to Bogie at("<<i<<")= " << times.at(i)<<std::endl;   
      Pose pose = sim->getFriendlyPose();
      pose.position = points.at(0);
      poses.push_back(pose);
    }
    sim->testPose(poses);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
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

    std::cout << "GetPostionOfBogie: points size = "<<points.size()<<std::endl;   
    for (int i = 0; i < points.size(); i++)
    {

      std::cout << "GetPostionOfBogie: points.at("<<i<<"): x = " << points.at(i).x << ", y = "<<points.at(i).y <<std::endl;   
    
    }
    points.clear();

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
    
   std::cout << "[" << sim->elapsed() / 1000 << "s]" << std::endl;
   std::cout << "Friendly {x, y, orientation}:"<< std::endl;
   std::cout << "  - x: " << pose.position.x << "m" << std::endl;
   std::cout << "  - y: " << pose.position.y << "m" << std::endl;
  // std::cout << "  - orient: " << pose.orientation*180/M_PI << " deg" << std::endl << std::endl;
   //std::this_thread::sleep_for(std::chrono::milliseconds(100));

//! The below commented out code get's rangeVelocity and rangeBearing
//! Should not be called from this thread as these are blocking calls (they stop this thread)

   std::vector<RangeVelocityStamped> rangeVeclocityStampedVec = sim->rangeVelocityToBogiesFromBase();
   std::vector<RangeBearingStamped> rangeBearingStampedVec = sim->rangeBearingToBogiesFromFriendly();
  //  for (auto rangeVeclocityStamped : rangeVeclocityStampedVec) {
  //    std::cout << "[" << rangeVeclocityStamped.timestamp << "] [R,V]=[" <<
  //                 rangeVeclocityStamped.range << ","  << rangeVeclocityStamped.velocity << "]" << std::endl;
  //  }

  //  for (auto rangeBearingStamped : rangeBearingStampedVec) {
  //    std::cout << "[" << rangeBearingStamped.timestamp << "] [R,the]=[" <<
  //                 rangeBearingStamped.range << ","  << rangeBearingStamped.bearing << "]" <<
  //                 " [x,y]=[" << rangeBearingStamped.range*cos(rangeBearingStamped.bearing) << "," <<
  //                               rangeBearingStamped.range*sin(rangeBearingStamped.bearing) << "]" << std::endl;
  //  }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

  }
}

void Control_aircraft:: controlThread(const std::shared_ptr<Simulator> & sim) {
  int offset = 0;
  int count = 0;
  while(true){
    //Feed the watchdog control timer
    double lin = sim->getFriendlyLinearVelocity() + offset;
    if(lin >= sim->V_MAX)
      {
        lin = sim->V_MAX;
        offset = 0;
      }
    double ang = sim->getFriendlyAngularVelocity() + offset;

    offset +=2;
    bool result = sim->controlFriendly(lin, ang);
    //std::cout << "V_max= " << sim->V_MAX <<", lin= "<<lin<<std::endl;
    //std::cout << "result set: " << result <<std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void Control_aircraft::ControlAircraft(int argc, char *argv[]){
           GameMode game_mode = GameMode::BASIC;
   //If code is started with --advanced, it will run in advanced mode
   std::cout << "Run with: " << argv[0] << " --advanced to run in advanced mode" << std::endl;
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
  std::cout << "Airspace Size:" << Simulator::AIRSPACE_SIZE << std::endl;


  threads.push_back(sim->spawn());
  threads.push_back(std::thread(controlThread, sim));
  //threads.push_back(std::thread(exampleThread, sim));
  threads.push_back(std::thread(EstimateTimeToPositionOfTheBogie, sim));

  //Join threads and end!
  for(auto & t: threads){
    t.join();
  }
}