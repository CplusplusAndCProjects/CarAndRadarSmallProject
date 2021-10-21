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
  
  std::vector<signed char> data_arr = map_.data;
  for (int i = 0; i < data_arr.size(); i++)
  {
    /* code */
    if(data_arr.at(i)==-1)
      count++;
  }
  

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
