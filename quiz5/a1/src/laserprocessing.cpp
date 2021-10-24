#include "laserprocessing.h"

LaserProcessing::LaserProcessing(sensor_msgs::LaserScan laserScan):
    laserScan_(laserScan)
{

}



//! @todo
//! TASK 1 - Refer to README.md and the Header file for full description
unsigned int LaserProcessing::countHighIntensity()
{
  unsigned int count=0;
  std::vector<float> intensitie_arr = laserScan_.intensities;
  for (int i = 0; i < intensitie_arr.size(); i++)
  {
      /* code */
      if (intensitie_arr.at(i)>0.5)
      {
          count ++;
          /* code */
      }
      
  }
  
  return count;
}

//! @todo
//! TASK 2 - Refer to README.md and the Header file for full description
unsigned int LaserProcessing::countSegments()
{
  unsigned int count=0;
  std::vector<float> intensitie_arr = laserScan_.intensities;
  std::vector<float> ranges_arr = laserScan_.ranges;


  for (int i = 0; i < intensitie_arr.size(); i++)
  {
      //std::cout<<"ranges["<<i<<"]= "<<ranges_arr.at(i)<<std::endl;
      /* code */
      if (intensitie_arr.at(i)>0.5 && ranges_arr.at(i)<0.3)
      {
          count ++;
          /* code */
      }
      
  }
  std::cout<<"count = "<<count<<std::endl;
  return count;
}



//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
geometry_msgs::Point LaserProcessing::detectPositionHighIntensity(){

    geometry_msgs::Point point;
    std::vector<float> intensitie_arr = laserScan_.intensities;
    //std::vector<float> ranges_arr = laserScan_.ranges;
    for (int i = 0; i < intensitie_arr.size(); i++)
    {
        //std::cout<<"ranges["<<i<<"]= "<<ranges_arr.at(i)<<std::endl;
        /* code */
        if (intensitie_arr.at(i)>0.5)
        {
            /* code */
            float angle = laserScan_.angle_min + laserScan_.angle_increment*i;// + angle_range/2;
            float range = laserScan_.ranges.at(i);
            point.x = static_cast<double>(range*cos(angle));
            point.y = static_cast<double>(range*sin(angle));
            break;
        }
        
    }
    std::cout<<"point.x= "<<point.x<<", point.y= "<<point.y<<std::endl;
    return point;
}


//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
geometry_msgs::Pose LaserProcessing::detectPoseHighIntensity(){


    geometry_msgs::Pose pose;
    geometry_msgs::Point point;
    geometry_msgs::Quaternion quaternion;
    std::vector<float> intensitie_arr = laserScan_.intensities;
    //std::vector<float> ranges_arr = laserScan_.ranges;
    for (int i = 0; i < intensitie_arr.size(); i++)
    {
        //std::cout<<"ranges["<<i<<"]= "<<ranges_arr.at(i)<<std::endl;
        /* code */
        if (intensitie_arr.at(i)>0.5)
        {
            /* code */
            float angle = laserScan_.angle_min + laserScan_.angle_increment*i;// + angle_range/2;
            float range = laserScan_.ranges.at(i);
            point.x = static_cast<double>(range*cos(angle));
            point.y = static_cast<double>(range*sin(angle));
            quaternion = tf::createQuaternionMsgFromYaw(abs(angle/(M_PI)));
            break;
        }
        
    }
    pose.position = point;
    pose.orientation = quaternion;

    return pose;
}

void LaserProcessing::newScan(sensor_msgs::LaserScan laserScan){
    laserScan_=laserScan;
}


geometry_msgs::Point LaserProcessing::polarToCart(unsigned int index)
{
    float angle = laserScan_.angle_min + laserScan_.angle_increment*index;// + angle_range/2;
    float range = laserScan_.ranges.at(index);
    geometry_msgs::Point cart;
    cart.x = static_cast<double>(range*cos(angle));
    cart.y = static_cast<double>(range*sin(angle));
    return cart;
}

double LaserProcessing::angleConnectingPoints(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return atan2(p2.y - p1.y, p2.x - p1.x);
}
