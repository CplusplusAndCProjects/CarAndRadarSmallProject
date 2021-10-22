#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"

#include "a4_setup/RequestGoal.h"

/**
 * This node shows some connections and publishing images
 */


class PfmsSample{

public:
  /*! @brief PfmsSample constructor.
   *
   *  Will take the node handle and initialise the callbacks and internal variables
   */
    PfmsSample(ros::NodeHandle nh);

  /*! @brief PfmsSample destructor.
   *
   *  Will tear down the object
   */
    ~PfmsSample();


  /*! @brief Request Goal service callback
   *
   *  @param req The requested goal.
   *  @param res The responce
   *
   *  @return bool - Will return true to indicate the request has sucsedded
   */
    bool requestGoal(a4_setup::RequestGoal::Request  &req,
             a4_setup::RequestGoal::Response &res);


  /*! @brief Odometry Callback
   *
   *  @param nav_msgs::OdometryConstPtr - The odometry message
   *  @note This function and the declaration are ROS specific
   */
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

  /*! @brief OccupancyGrid Callback
    *
    *  @param sensor_msgs::ImageConstPtr - The imageconst message
    *  @note This function and the declaration are ROS specific
    */
     void occupancyGridCallback(const nav_msgs::OccupancyGridPtr & msg);

  /*! @brief seperate thread.
   *
   *  The main processing thread that will run continously and utilise the data
   *  When data needs to be combined then running a thread seperate to callback will gurantee data is processed
   */

    void seperateThread(); 

private:
    ros::NodeHandle nh_;

    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    ros::ServiceServer service_;

    std::atomic<bool> poseRequested_;/*!< The boolean to indicate if we have recieved a request */

    //! Question: Is theer a better way to store data, instead of a structure?
    //! refer to Tutorial 7 exercise 3
    struct PoseDataBuffer
    {
      //! Question: Given these elements come in two's (pose and time)
      //! Is there a better type of STL container rather than two seperate deques?
        geometry_msgs::Pose pose;
        std::mutex mtx;
    };
    PoseDataBuffer poseDataBuffer_;//! Container for pose data
    PoseDataBuffer requestedPoseDataBuffer_; //! Container for the requested pose data

    struct OgMapBuffer
    {
        //! Question: Given these elements come in two's (image and time)
        //! Is there a better type of STL container rather than two seperate deques?
        nav_msgs::OccupancyGrid grid;
        std::mutex mtx;
    };

    OgMapBuffer ogMapBuffer_;//! Container for image data


};

