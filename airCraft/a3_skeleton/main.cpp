/*! @file
 *
 *  @brief Main entry point for assignment 3.
 *  This assignment focus mainly on threading and estimating on 2D space
 *  Deliverable: Drive the Friendly Aircarft in AirSpace to catch as much Enemy Bogie as possible
 * 
 *  Please follow this instruction to start the Assignment task
 *  + Initialise all Estimating Threads and Feed the WATCHDOG TIMER
 *  + Initialise FriendlyRadar and Base Radar and push them to the container
 *  + Initialise the Sharepointer to get access to all the data inside all constructor
 *  + Initialise the main thread, call the spawn function to spawn the bogie
 *  + Push all the nesessary threads in the main thread to start
 *  /***********************************************************************\
 *
 *  @author TRONG DUY BUI - 13432190
 *  @date 2/10/2020
*/
#include <thread>
#include <vector>
#include <iostream>
#include "simulator.h"
#include "friendly_radar.h"
#include "base_radar.h"
#include "synchronization.h"

using namespace std;

void EstimationThread(const std::shared_ptr<Simulator> & sim, const std::shared_ptr<Synchronization> & datapointer) {
  while(true) {
    datapointer->delay(datapointer->TIME_BETWEEN_POSE);
    //Get the friendly aircraft's position and orientation
    vector<Pose> pose = datapointer->Bogie_Estimation();

    sim->testPose(pose);
    cout << "[" << sim->elapsed() / 1000 << "s]" << std::endl;
    for(int i=0; i<pose.size();i++){
      cout << "Bogie" <<"[" << i <<"] " << "{x, y, orientation}:"<< " ";
      cout << "  - x: " << pose[i].position.x << "m" << " ";
      cout << "  - y: " << pose[i].position.y << "m" << " ";
      cout << "  - orient: " << pose[i].orientation << " radians" << std::endl << std::endl;
    }
  }
}

//This thread will simply get the current velocity and feed it back into
//controlled velocity, at the designated minimum time required (watchdog time) refer
//'controlFriendly()' documentation in the simualtor class.

void PurePursuitThread(const std::shared_ptr<Simulator> & sim, const std::shared_ptr<Synchronization> & datapointer) {
    while(true){
      //Feed the watchdog control timer
      datapointer->delay(datapointer->WATCHDOG_TIMER);
      double friendly_linear_velocityocity = datapointer->Pure_Pursuit(sim).linear_velocity;
      double friendly_angular_velocity = datapointer->Pure_Pursuit(sim).angular_velocity;
      
      sim->controlFriendly(friendly_linear_velocityocity, friendly_angular_velocity);

    }
}


//This thread will get the data in form of RangeVelocityStamped type and feed the
void BogieBase_Producer(const std::shared_ptr<Synchronization> & datapointer) {
    while(true){
      //Feed the watchdog control timer
      datapointer->BogieBaseRangeProducer();
    }
}


//This thread will get the data in form of RangeBearingStamped type and feed the
void BogieFriendly_Producer(const std::shared_ptr<Synchronization> & datapointer) {
    while(true){
      //Feed the watchdog control timer
      datapointer->BogieFriendlyRangeProducer();
    }
}



int main(void){

    vector<std::thread> threads;

    //setup the share pointer for all threads
    shared_ptr<Simulator> sim(new Simulator());
    
    //Initialise Base Radar
    Base_Radar Base_Radar(sim);

    //Initialise Aircraft radat
    Friendly_Radar Friendly_Radar(sim);

    //set up the constructor to store all initialised radar
    vector<Radar_Interface*> radar_container;

    //push 2 radars to the container
    radar_container.push_back(&Friendly_Radar);
    radar_container.push_back(&Base_Radar);

    //Initialise fusion constructor
    Synchronization synchronization(radar_container,sim);

    //setup the share pointer for the fusion
    shared_ptr<Synchronization> DataPointer(new Synchronization(radar_container,sim));

    //push all the thread to the main thread
    threads.push_back(sim->spawn());
    threads.push_back(thread(PurePursuitThread, sim, DataPointer));
    threads.push_back(thread(EstimationThread, sim, DataPointer));
    threads.push_back(thread(BogieBase_Producer, DataPointer));
    threads.push_back(thread(BogieFriendly_Producer, DataPointer));

  //Join threads and begin!
  while(true){
    for(auto & t: threads){
        t.join();
    }
  }
  return 0;
}