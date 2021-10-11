
#include "tf2.h"
#include "tf.h"
#include <cmath> // for trig operations
#include<types.h>
namespace tf2 {

    //! @todo
    //! TASK 1 - Refer to README.md and the Header file for full description
    Point local2Global(RangeBearingStamped rangeBearing, Pose aircraft)
    {
      Point p;
        // caculatetor euler info
      double yaw, pitch, roll;
      double t0 = 2.0 * (aircraft.orientation.w * aircraft.orientation.x + aircraft.orientation.y * aircraft.orientation.z);
      double t1 = 1.0 - (2.0 * (aircraft.orientation.x * aircraft.orientation.x + aircraft.orientation.y * aircraft.orientation.y));
      roll = atan2(t0,t1);
      double t2 = 2.0 * (aircraft.orientation.w * aircraft.orientation.y + aircraft.orientation.x * aircraft.orientation.z);
      if(t2 >1.0)
      {
        t2 = 1.0;
      }
      else if(t2<-1.0)
      {
        t2 = -1.0;
      }

      pitch = asin(t2);
      double t3 = 2.0 * (aircraft.orientation.w * aircraft.orientation.z + aircraft.orientation.y * aircraft.orientation.x);
      double t4 = 1.0 - 2.0 * (aircraft.orientation.y * aircraft.orientation.y + aircraft.orientation.z * aircraft.orientation.z);
      yaw = CaculateOrientation(aircraft);
    
      double theta = normaliseAngle(yaw + rangeBearing.bearing);
      p.x = aircraft.position.x+rangeBearing.range*cos(theta);
      p.y = aircraft.position.y+rangeBearing.range*sin(theta);
      return p;
    }

    //! @todo
    //! TASK 2 - Refer to README.md and the Header file for full description
    RangeBearingStamped global2local(Point bogie, Pose aircraft)
    {
        RangeBearingStamped rbstamped = {0, 0,0};


          double x_bogie = bogie.x;
          double y_bogie =bogie.y;
          double x_fr = aircraft.position.x;
          double y_fr = aircraft.position.y;
          double alpha_fr  = tf::quaternionToYaw(aircraft.orientation);//CaculateOrientation(aircraft);

          double range =  sqrt(pow(x_bogie-aircraft.position.x, 2) + pow(y_bogie-aircraft.position.y, 2));
          double total_angle = atan2(bogie.y -aircraft.position.y, bogie.x -aircraft.position.x);
          if (total_angle <0 ) 
            {
              total_angle+=2*M_PI;
            }
          double bearing = total_angle - alpha_fr;
          // if (bearing < -2*M_PI)
          //   bearing+=2*M_PI;
          double bearing_drgrees = bearing *180/M_PI;
          if(bearing_drgrees < -180 || bearing_drgrees > 180)
            bearing_drgrees = 360 - abs(bearing_drgrees);
          bearing = bearing_drgrees * M_PI/180;
          // The below creates a rangeBearing (which is a struct) with range =0, bearing =0 and timestamp =0
          // (the timestamp is not relevant here to us now, but in ROS it will be)
          rbstamped = {range, bearing, 0};
      
        return rbstamped;

    }
    double CaculateOrientation(Pose aircraft) {
      double yaw ;
      double t3 = 2.0 * (aircraft.orientation.w * aircraft.orientation.z + aircraft.orientation.y * aircraft.orientation.x);
      double t4 = 1.0 - 2.0 * (aircraft.orientation.y * aircraft.orientation.y + aircraft.orientation.z * aircraft.orientation.z);
      yaw = atan2(t3, t4);
      return yaw;
    }

    double normaliseAngle(double theta) {
      if (theta > (2 * M_PI))
        theta = theta - (2 * M_PI);
      else if (theta < 0)
        theta = theta + (2 * M_PI);

      return theta;
    }

}
