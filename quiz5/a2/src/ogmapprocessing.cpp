#include "ogmapprocessing.h"
#include <ros/console.h>

OgmapProcessing::OgmapProcessing(nav_msgs::OccupancyGrid map) :
  map_(map)
{

}

//! @todo
//! TASK 1 - Refer to README.md and the Header file for full description
unsigned int OgmapProcessing::countUnknwonCells(){

  unsigned int count = 0;

  return count;

}

//! @todo
//! TASK 2 - Refer to README.md and the Header file for full description
bool OgmapProcessing::isLocationFree(geometry_msgs::Point goal){

  bool free =false;
  return free;

}

void OgmapProcessing::newMap(nav_msgs::OccupancyGrid map){
  map_=map;
}
