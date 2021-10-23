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
  int row_req, col_req, row_rob, col_rob;
  geometry_msgs::Pose pose_origin = map_.info.origin;
  int index_req, index_rob;
  row_rob = (int)(abs(goal.x - pose_origin.position.x))/map_.info.resolution;
  col_rob = (int)(abs(goal.y - pose_origin.position.y))/map_.info.resolution;

  if((col_rob>map_.info.width) && (row_rob>map_.info.height))
    free = true;
  else
    free = false;

  //free != checkcellInside;
  return free;

}

void OgmapProcessing::newMap(nav_msgs::OccupancyGrid map){
  map_=map;
}
