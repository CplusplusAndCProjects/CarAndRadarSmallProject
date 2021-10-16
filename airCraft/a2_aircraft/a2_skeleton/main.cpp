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
#include"control_aircraft.h"

using namespace std;
using namespace simulator;
int main(int argc, char *argv[]){

  //Run(argc, argv);
  //std::shared_ptr<Simulator> sim;
  Control_aircraft *control_aircraft = new Control_aircraft();
  control_aircraft->ControlAircraft(argc, argv);
  return 0;
}