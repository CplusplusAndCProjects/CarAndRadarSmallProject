#include "sample.h"
#include <cmath>


/**
 * This node shows some connections and publishing images
 */


PfmsSample::PfmsSample(ros::NodeHandle nh)
    : nh_(nh)
{
    //Subscribing to odometry (Do we need to change thge topic???)
    sub1_ = nh_.subscribe("/robot_0/odom", 1000, &PfmsSample::odomCallback,this);

    //Subscribing to occupnacy grid
    sub2_ = nh_.subscribe("local_map/local_map", 1, &PfmsSample::occupancyGridCallback,this);

    //Allowing an incoming service on /check_goal
    service_ = nh_.advertiseService("check_goal", &PfmsSample::requestGoal,this);
}

PfmsSample::~PfmsSample()
{

}


bool PfmsSample::requestGoal(a4_setup::RequestGoal::Request  &req,
             a4_setup::RequestGoal::Response &res)
{
  std::unique_lock<std::mutex> lck1 (ogMapBuffer_.mtx);
  std::unique_lock<std::mutex> lck2 (requestedPoseDataBuffer_.mtx);

  //The incoming request is a Global coordinate.
  ROS_INFO_STREAM("requested goal [x,y,yaw]=[" << req.pose.x <<"," <<req.pose.y << "]");

  //! @todo : 4 - return true if the global coordinate supplied in the requst is within current OgMap
  //! (just needs to be within the map - either free / occupied or unknown)
  //!
  //! To find information about the service execute "rossrv info a4_setup/RequestGoal"
  //!
  //! Save the pose so we can use it in the seperateThread (think how we can share data between functions?)
  //!
  // Return in the res.ack
  // true : if the global coordinate is in the current OgMap
  // false : global coordinate outside of current OgMap

  // Informion you have now:
  // poseDataBuffer_      - position of Robot (refer code in seperateThread
  //                        on how to get x,y,yaw of robot
  // ogMapDataBuffer_     - ogMap, size can be accessed via meta data

  //res.ack = is the global coordinate in the current OgMap;
  //ROS_INFO_STREAM("sending back response:" << res.ack);

  if ((fabs(req.pose.x) < (ogMapBuffer_.grid.info.width/2))
          && (fabs(req.pose.y) < (ogMapBuffer_.grid.info.height/2))) {
      res.ack = true;
  }

  else {
      res.ack = false;
  }

  requestedPoseDataBuffer_.pose.position.x = req.pose.x;
  requestedPoseDataBuffer_.pose.position.y = req.pose.y;

  ROS_INFO_STREAM("sending back response:" << res.ack);
  return true;
}

void PfmsSample::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    //! REMEBER: on command line you can view entier msg as
    //! rosmsg show nav_msgs/Odometry
    std::unique_lock<std::mutex> lck (poseDataBuffer_.mtx);
    poseDataBuffer_.pose = msg->pose.pose;
}



void PfmsSample::occupancyGridCallback(const nav_msgs::OccupancyGridPtr& msg)
{

  std::unique_lock<std::mutex> lck (ogMapBuffer_.mtx);
  ogMapBuffer_.grid = *msg;

}


void PfmsSample::seperateThread() {
   /**
    * The below loop runs until ros is shutdown, to ensure this thread does not remain
    * a zombie thread
    *
    * The loop locks the buffer, checks the size
    * And then pulls items: the pose and timer_t
    * You can contemplate weather these could have been combined into one ...
    */

   geometry_msgs::Pose pose;
   geometry_msgs::Pose pose_req;
   nav_msgs::OccupancyGrid grid;
   int numCells=0;

    //! @todo : 2 - Adjust rate limiter to run code every 5 seconds
    //!
    //! Additional Question
    //! What is rate_limiter?
    //! - rate_limiter is the object we have created of ros::Rate. The constructor requires us to enter a desired rate to run at (frequency) in Hz
    ros::Rate rate_limiter(0.2);

    while (ros::ok()) {

      if(poseRequested_){
        //! Get the Pose message
        poseDataBuffer_.mtx.lock();
        pose=poseDataBuffer_.pose;
        poseDataBuffer_.mtx.unlock();

        //! Get the OgMap message
        ogMapBuffer_.mtx.lock();
        grid= ogMapBuffer_.grid;
        ogMapBuffer_.mtx.unlock();

        requestedPoseDataBuffer_.mtx.lock();
        pose_req = requestedPoseDataBuffer_.pose;
        requestedPoseDataBuffer_.mtx.unlock();

        ROS_INFO_STREAM(pose);

        //! @todo : 5 - The number of cells between the robot pose and the global coordinate requested
        //!
        //! To do So:
        //!
        //! - Check if the coordinate is on OgMap, and convert it to a local robot coordinate here)
        //! - Determine the number of cells (we have the resolution that should assist in counting the distance)

        geometry_msgs::Point origin = grid.info.origin.position;


        int row_req, col_req, row_rob, col_rob;
        int index_req, index_rob;
        row_req = (int)(pose_req.position.x - origin.x)/grid.info.resolution;
        col_req = (int)(pose_req.position.y - origin.y)/grid.info.resolution;
        row_rob = (int)(pose.position.x - origin.x)/grid.info.resolution;
        col_rob = (int)(pose.position.y - origin.y)/grid.info.resolution;


        if((row_req >=0) && (col_req >=0)  && (col_req<grid.info.width) && (row_req<grid.info.height)){
            index_req = row_req * grid.info.width + col_req;
        }
        else {
            ROS_INFO_STREAM("Requested coordinate outside of the OgMap!");
        }

        if((row_rob >=0) && (col_rob >=0)  && (col_rob<grid.info.width) && (row_rob<grid.info.height)){
            index_rob = row_rob * grid.info.width + col_rob;
        }

        else {
            ROS_INFO_STREAM("Robot coordinate outside of the OgMap!");
        }

        int numCells = abs(index_rob-index_req);

        ROS_INFO_STREAM("Number of cells between robot and the global coordinate requested is:" << numCells);
    }

        rate_limiter.sleep();
    }
}

