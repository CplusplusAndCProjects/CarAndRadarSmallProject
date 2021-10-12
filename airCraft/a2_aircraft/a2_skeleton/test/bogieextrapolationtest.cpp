#include "gtest/gtest.h"
#include <iostream>

#include <vector>
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

using namespace std;
using geometry_msgs::Pose;
using geometry_msgs::RangeBearingStamped;
using std::vector;
using namespace tf;
using namespace tf2;

//==================== UNIT TEST START ====================//
static std::vector<RangeVelocityStamped> rangeVelocityStampeds; 
void controlThread(const std::shared_ptr<Simulator> & sim, int timer) {
  while(timer){
    //Feed the watchdog control timer
    double lin = sim->getFriendlyLinearVelocity();
    double ang = sim->getFriendlyAngularVelocity();
    std::cout << "Advanced Mode Activated" << std::endl;
    sim->controlFriendly(lin, ang);
    timer--;
    if (timer <= 0)
      {
          sim->stop();
          //break;
      }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  
}
void exampleThread(const std::shared_ptr<Simulator> & sim) {
  while(true) {
    //Get the friendly aircraft's position and orientation
    Pose pose = sim->getFriendlyPose();
    std::vector<Pose> poses;
    poses.push_back(pose);
    sim->testPose(poses);
    rangeVelocityStampeds = sim->rangeVelocityToBogiesFromBase();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
}
TEST(BogieTest, DetermineVelocity)
{

  {
    GameMode game_mode = GameMode::BASIC;
    std::vector<std::thread> threads;

    //Create a shared pointer for the simulator class
    std::shared_ptr<Simulator> sim(new Simulator(false,game_mode));
    int time = 6;
    threads.push_back(sim->spawn());
    threads.push_back(std::thread(controlThread, sim,3));
    std::vector<RangeVelocityStamped> rangeVelocityStampeds; 
    for(auto & t: threads){
      t.join();
      }

    for (size_t i = 0; i < rangeVelocityStampeds.size(); i++)
    {
      EXPECT_NEAR(rangeVelocityStampeds[i].velocity,10,0.5);
    }

  }

  {
    GameMode game_mode = GameMode::ADVANCED;//GameMode::BASIC;
    std::vector<std::thread> threads;

    //Create a shared pointer for the simulator class
    std::shared_ptr<Simulator> sim(new Simulator(false,game_mode));
    int time = 6;
    threads.push_back(sim->spawn());
    threads.push_back(std::thread(controlThread, sim,3));
    std::vector<RangeVelocityStamped> rangeVelocityStampeds; 
    for(auto & t: threads){
      t.join();
      }

    for (size_t i = 0; i < rangeVelocityStampeds.size(); i++)
    {
      EXPECT_NEAR(rangeVelocityStampeds[i].velocity,0,0.5);
    }

  }

}

TEST(BogieTest, ExtrapolationInTime)
{
  
  {
    GameMode game_mode = GameMode::BASIC;
    std::vector<std::thread> threads;

    //Create a shared pointer for the simulator class
    std::shared_ptr<Simulator> sim(new Simulator(false,game_mode));
    int time = 6;
    threads.push_back(sim->spawn());
    threads.push_back(std::thread(controlThread, sim,3));
    std::vector<RangeVelocityStamped> rangeVelocityStampeds; 
    for(auto & t: threads){
      t.join();
      }

    for (size_t i = 0; i < rangeVelocityStampeds.size(); i++)
    {
      EXPECT_NEAR(rangeVelocityStampeds[i].velocity,10,0.5);
    }

  }

  {
    GameMode game_mode = GameMode::ADVANCED;//GameMode::BASIC;
    std::vector<std::thread> threads;

    //Create a shared pointer for the simulator class
    std::shared_ptr<Simulator> sim(new Simulator(false,game_mode));
    int time = 6;
    threads.push_back(sim->spawn());
    threads.push_back(std::thread(controlThread, sim,3));
    std::vector<RangeVelocityStamped> rangeVelocityStampeds; 
    for(auto & t: threads){
      t.join();
      }

    for (size_t i = 0; i < rangeVelocityStampeds.size(); i++)
    {
      EXPECT_NEAR(rangeVelocityStampeds[i].velocity,0,0.5);
    }

  }


}



int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}