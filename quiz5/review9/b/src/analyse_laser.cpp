#include "ros/ros.h"

//ROS Message Types
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <iostream>


/**
 * This node attempts to draw the received path on an image
 */


class PfmsSample{

    ros::NodeHandle nh_;
    ros::Subscriber sub1_;


public:
    PfmsSample(ros::NodeHandle nh)
    : nh_(nh)
    {
        sub1_ = nh_.subscribe("robot_0/base_scan", 10, &PfmsSample::laserCallback,this);
    }

    ~PfmsSample()
    {
    }


    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
    {

      /**
       * @todo - Ex 4 : Find the number of high intensity laser readings
       *
       * On command line type 'rosmsg show sensor_msgs/LaserScan'
       * What are we provided in this message?
       * What part of the message do we need to iterate over?
       */

       //  ROS_INFO_STREAM("Total number of high intensity readings");


      /**
       * @todo - Ex 5 : Find a group of high intensity readings with neighbours within 0.3m of each other + print the x,y location
       *      of the first point and last point in this group of points
       *
       * HINTS:
       * - Iterate through each laser readings, checking the range and intensity
       * - Check that the reading is high intensity and the neighbour (next cells is withion 0.3m of that cell
       * - Print the x,y location of the first point and last point in this group of points
       */

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
  ros::init(argc, argv, "analyse_laser");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;


  /**
   * Let's start create object of PfMSSample
   * and thereafter start the thread on teh function desired
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

