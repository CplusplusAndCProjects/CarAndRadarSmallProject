#include "ros/ros.h"

//ROS Message Types
#include <nav_msgs/OccupancyGrid.h>

#include <sstream>
#include <iostream>
#include <string>


/**
 * This node attempts to draw the received path on an image
 */


class PfmsSample{

    ros::NodeHandle nh_;
    ros::Subscriber sub1_;
    ros::Publisher map_publisher_;

public:
    PfmsSample(ros::NodeHandle nh)
    : nh_(nh)
    {
        sub1_ = nh_.subscribe("/local_map/local_map", 1, &PfmsSample::mapCallback,this);
        map_publisher_ = nh.advertise<nav_msgs::OccupancyGrid>("new_map", 1, true);
    }

    ~PfmsSample()
    {
    }


    void mapCallback(const nav_msgs::OccupancyGridPtr& map)
    {

      int size_x = map->info.width;
      int size_y = map->info.height;

      //Here we generate a new OgMap message
      nav_msgs::OccupancyGrid newGrid;
      newGrid.header = map->header; // we copy the header
      newGrid.info = map->info; // and the map info
      newGrid.data.assign(size_x * size_y, -1); //And we create a vector os size_x * size_y values, with each element of vector assigned value -1

      const std::vector<int8_t>& map_data (map->data); // In this step we now have the newGrid actual cell data inside new_map_data
      std::vector<int8_t>& new_map_data (newGrid.data); // In this step we now have the newGrid actual cell data inside new_map_data



      //! @todo : 2 - count and display the number of uknown cells.
      //!
      //! To do So:
      //!
      //! - Check the below code that loops through cells, modify the code if needed
      //! - Determine the number of unknwon cells (and increment if the cells is uknown)
      //! - USE ROS_INFO_STREAM to print the total number of unknwon cells


      //! @todo : 3 - publish an augument new OgMap (newGrid) that has all unknown cells converted to 50% free (value of 50).
      //!
      //! To do So:
      //!
      //! - Find the cells that are unknown (done in task 2)
      //! - Change the value of new_map_data  (which is the data inside newGridMap)
      //! - To change the value of the data use new_map_data.at(cell_location) where the cell location is absolute value from 0,0
      //! we need to add the total number of cells skipped (number of cells in each row and then the number of cells in that row)

        // The outer loop goes by row
        for (int y = 1; y< size_y-1; ++y){

          // Since we are dealinbg with a row major data storage, idx_map_y will give us the total number of cells that we skipped to
          // get to row y
          int idx_map_y = size_x * y;

          // The inner loop goes by column
          for (int x = 1; x < size_x-1; ++x){

            //We now switch based on the value of the cell at position idx_map_y + x
            switch (map_data[idx_map_y + x])
            {
            case -1:
              break;

            case 0:
              new_map_data.at(idx_map_y + x)=0;
              break;

            case 100:
              break;
            }
          }

        }

        map_publisher_.publish(newGrid);

      }
};


int main(int argc, char **argv)
{


  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "generate_ogmap");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  /**
   * Let's create object of PfmsSample
   */
  std::shared_ptr<PfmsSample> gc(new PfmsSample(nh));

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();

  return 0;
}

