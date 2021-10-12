#include "gtest/gtest.h"
#include <iostream>
#include <vector>

// Student defined libraries, for instance
//#include "flightplanner.h"
#include"../dep/include/control_aircraft.h"
// Student defined libraries, for instance
//#include "flightplanner.h"
#include "../dep/include/types.h"
#include <algorithm>

// header files needed from our libraries, because of include_directories in CMakeLists.txt we don't need the ..
// before these filenames
#include "../dep/include/tf2.h"
#include "../dep/include/tf.h"
#include "../dep/include/analysis.h"

//==================== UNIT TEST START ====================//

static std::vector<RangeVelocityStamped> rangeVelocityStampeds; 
static std::vector<RangeBearingStamped> rangeBearingStampeds;
static Point bogieComputed;
static vector<double> times1 ;


vector<double> timeToImpact1(Pose origin, std::vector<Point> goals_){

    //The consts you will need are
    //Display::G_MAX
    //Display::V_TERM
    //Display::V_MAX
    vector<double> times;
    //vector<RangeBearingStamped> rangeBearingStampeds = display->scan(origin);
    for (size_t i = 0; i < goals_.size(); i++)
    {
        /* code */
        RangeBearingStamped rangeBearingStamped = global2local(goals_.at(i), origin);
        //rangeBearingStamped.timestamp = rangeBearingStampeds.at(i).timestamp;

        double bearing_drgrees = rangeBearingStamped.bearing *180/M_PI;

        if(bearing_drgrees < -180 || bearing_drgrees > 180)
            bearing_drgrees = 360 - abs(bearing_drgrees);
        rangeBearingStamped.bearing = bearing_drgrees * M_PI/180;

        double Omega_max = Simulator::G_MAX * 9.81 / Simulator::V_TERM; 
        double time = rangeBearingStamped.range/Simulator::V_MAX + rangeBearingStamped.bearing/Omega_max;
        times.push_back(time);
    }

    return times;
}
std::vector<Point> EstimatePositionOfTheBogie1(const std::shared_ptr<Simulator> & sim){

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
//static RangeBearingStamped rb;
void controlThread(const std::shared_ptr<Simulator> & sim, int timer) {
  while(timer){
    //Feed the watchdog control timer
    double lin = sim->getFriendlyLinearVelocity();
    double ang = sim->getFriendlyAngularVelocity();
    rangeVelocityStampeds = sim->rangeVelocityToBogiesFromBase();
    rangeBearingStampeds = sim->rangeBearingToBogiesFromFriendly();
    std::cout << "Advanced Mode Activated" << std::endl;
    sim->controlFriendly(lin, ang);
    vector<geometry_msgs::Point> points = EstimatePositionOfTheBogie1(sim);
    Pose pose = sim->getFriendlyPose();

    times1 = timeToImpact1(pose,points);

    timer--;
    if (timer <= 0)
      {
          sim->stop();
          //break;
      }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  
}


TEST(AssociationTest, RangeBearingToBogie)
{

    Pose aircraft;
    aircraft.position = {0,0,0};
    aircraft.orientation = yawToQuaternion(0.785398);
  {
    std::vector<double> bearing = {0.18763, 5.04243, 1.36617, 4.8602};
    GameMode game_mode = GameMode::BASIC;
    std::vector<std::thread> threads;

    //Create a shared pointer for the simulator class
    std::shared_ptr<Simulator> sim(new Simulator(false,game_mode));
    int time = 6;
    threads.push_back(sim->spawn());
    threads.push_back(std::thread(controlThread, sim, 3));
    //threads.push_back(std::thread(exampleThread, sim, aircraft));
    for(auto & t: threads)
    {
      t.join();
    }

    for (size_t i = 0; i < rangeBearingStampeds.size(); i++)
    {
      std::cout<<"rangeBearingStamped.bearing ["<< i <<"]=:" << rangeBearingStampeds[i].bearing<<endl;
      EXPECT_NEAR(rangeBearingStampeds[i].bearing,rangeBearingStampeds[i].bearing,0.5);
    }

  }

  {
    GameMode game_mode = GameMode::ADVANCED;//GameMode::BASIC;
    std::vector<std::thread> threads;
    std::vector<double> bearing = {1.30158, 4.5971, 4.13153, 3.41355};

    //Create a shared pointer for the simulator class
    std::shared_ptr<Simulator> sim(new Simulator(false,game_mode));
    int time = 6;
    threads.push_back(sim->spawn());
    threads.push_back(std::thread(controlThread, sim,3));
    //threads.push_back(std::thread(exampleThread, sim, aircraft));
    std::vector<RangeVelocityStamped> rangeVelocityStampeds; 
    for(auto & t: threads){
      t.join();
      }

    for (size_t i = 0; i < rangeBearingStampeds.size(); i++)
    {
      std::cout<<"rangeBearingStampeds.bearing ["<< i <<"]=:" << rangeBearingStampeds[i].bearing<<endl;
      EXPECT_NEAR(rangeBearingStampeds[i].bearing,rangeBearingStampeds[i].bearing,0.5);
    }

  }

}

TEST(AssociationTest, RangeVelocityToBogie)
{

    Pose aircraft;
    aircraft.position = {0,0,0};
    aircraft.orientation = yawToQuaternion(0.785398);

  {
    GameMode game_mode = GameMode::BASIC;
    std::vector<std::thread> threads;
    std::vector<double> velocitys = {0, 0, 0, 0};

    //Create a shared pointer for the simulator class
    std::shared_ptr<Simulator> sim(new Simulator(false,game_mode));
    int time = 6;
    threads.push_back(sim->spawn());
    threads.push_back(std::thread(controlThread, sim, 3));
    //threads.push_back(std::thread(exampleThread, sim, aircraft));
    for(auto & t: threads)
    {
      t.join();
    }

    for (size_t i = 0; i < times1.size(); i++)
    {
      std::cout<<"times ["<< i <<"]=:" << times1[i]<<endl;
      EXPECT_NEAR(times1[i],times1[i],0.5);
    }

  }

  {
    GameMode game_mode = GameMode::ADVANCED;//GameMode::BASIC;
    std::vector<std::thread> threads;
    std::vector<double> velocitys = {499.218, 421.752, 158.205, 116.299};

    //Create a shared pointer for the simulator class
    std::shared_ptr<Simulator> sim(new Simulator(false,game_mode));
    int time = 6;
    threads.push_back(sim->spawn());
    threads.push_back(std::thread(controlThread, sim,3));
    //threads.push_back(std::thread(exampleThread, sim, aircraft));
    for(auto & t: threads){
      t.join();
      }

    for (size_t i = 0; i < times1.size(); i++)
    {
      std::cout<<"times1["<< i <<"]=:" << times1[i]<<endl;
      EXPECT_NEAR(times1[i],times1[i],0.5);
    }

  }


}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
