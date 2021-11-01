#include "ogmapprocessing.h"
#include <ros/console.h>
#include"math.h"
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
  geometry_msgs::Pose pose_origin = map_.info.origin;
  std::vector<int> index_Free_arr;
  std::vector<signed char> data_arr = map_.data;
  int index_Free = 0;
  for (int i = 0; i < data_arr.size(); i++)
  {
    /* code */
    if(data_arr.at(i)== 0)
      index_Free_arr.push_back(i);
  }
  
//  std::cout<<"index free is"<<std::endl;
  // for (int i = 0; i < index_Free_arr.size(); i++)
  // {
  //   /* code */
  //     std::cout<<"index_Free_arr["<<i<<"] = "<<index_Free_arr[i]<<std::endl;

  // }
  
     // We compute the row and column as the distance from the origin divided by resolution
  int row = static_cast<int>((goal.x - pose_origin.position.x)/map_.info.resolution);
  int col = static_cast<int>((goal.y - pose_origin.position.y)/map_.info.resolution);
  //int index_test = map_.info.width*(row -1) + col;
  int index_test = map_.info.height*(col -1) + row;

    for (int i = 0; i < index_Free_arr.size(); i++)
  {
    /* code */
    if(index_test == index_Free_arr[i]){
      std::cout<<"location is free at cell: "<<index_Free_arr[i]<<std::endl;
      free = true;
      break;
    }

  }
  return free; 

}

void OgmapProcessing::newMap(nav_msgs::OccupancyGrid map){

  map_=map;


}

int OgmapProcessing::midPointCircleDetectOccupied(int rmax)
{

  for(int r=0;r<=rmax;r++){
      int x = r, y = 0;

      int index =0;

      if (testCell(x,y,index)){
        return index;
      }

      // When radius is zero only a single
      // point will be examined
      if (r > 0)
      {
          if (testCell(x,-y,index)){
            return index;
          }

          if (testCell(x,y,index)){
            return index;
          }

          if (testCell(-x,x,index)){
            return index;
          }

      }

      // Initialising the value of P
      int P = 1 - r;
      while (x > y)
      {
          y++;

          // Mid-point is inside or on the perimeter
          if (P <= 0)
              P = P + 2*y + 1;
          // Mid-point is outside the perimeter
          else
          {
              x--;
              P = P + 2*y - 2*x + 1;
          }

          // All the perimeter points have already been printed
          if (x < y)
              break;

          if (testCell(x,y,index)){
            return index;
          }
          if (testCell(-x,y,index)){
            return index;
          }
          if (testCell(x,-y,index)){
            return index;
          }
          if (testCell(-x,-y,index)){
            return index;
          }

          // If the generated point is on the line x = y then
          // the perimeter points have already been visited
          if (x != y)
          {
            if (testCell(y,x,index)){
              return index;
            }
            if (testCell(-y,x,index)){
              return index;
            }
            if (testCell(y,-x,index)){
              return index;
            }
            if (testCell(-y,-x,index)){
              return index;
            }


          }
      }
  }

  return -1;

}

bool OgmapProcessing::testCell(int x, int y, int& index){

  unsigned int col =x + map_.info.height/2;
  unsigned int row =y + map_.info.width/2;
  index = row * map_.info.width + col;
  return (map_.data.at(index) == 0) ? true : false; //Return true if value 100 and false otherwise;
}
