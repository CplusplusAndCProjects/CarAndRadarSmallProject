/*! @file
 *
 *  @brief Main entry point for assignment 2.
 *
 *  TODO: Add information here
 *
 *  @author {TODO: Your student name + id}
 *  @date {TODO}
*/
#include <thread>
#include <vector>
#include <iostream>
#include "control_aircraft.h"
int main(int argc, char *argv[])
{

   Control_aircraft *control_aircraft = new Control_aircraft();
   control_aircraft->ControlAircraft(argc, argv);

  return 0;
}
