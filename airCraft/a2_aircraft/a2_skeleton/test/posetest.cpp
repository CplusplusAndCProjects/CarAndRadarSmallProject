#include "gtest/gtest.h"
#include <iostream>

#include <vector>

// Student defined libraries, for instance
//#include "flightplanner.h"


#include "../dep/include/types.h"
#include <algorithm>

// header files needed from our libraries, because of include_directories in CMakeLists.txt we don't need the ..
// before these filenames
#include "../dep/include/tf2.h"
#include "../dep/include/tf.h"
#include "../dep/include/analysis.h"

using namespace std;
using geometry_msgs::Pose;
using geometry_msgs::RangeBearingStamped;
using std::vector;
using namespace tf;
using namespace tf2;
//==================== UNIT TEST START ====================//

TEST(PoseTest, LocalToGlobal)
{

    Pose aircraft;
    aircraft.position = {0,0,0};
    aircraft.orientation = yawToQuaternion(0.785398);

    {
        Point bogie = {3171.34,-4574.67,0};
        RangeBearingStamped rb = {5566.41,4.53316,0};
        Point bogieComputed = local2Global(rb,aircraft);
        EXPECT_NEAR(bogie.x,bogieComputed.x,0.5);
        EXPECT_NEAR(bogie.y,bogieComputed.y,0.5);
    }

    {
        Point bogie = {-3288.52,-2516.65,0};
        RangeBearingStamped rb = {4141,3.0094,0};
        Point bogieComputed = local2Global(rb,aircraft);
        
        EXPECT_NEAR(bogie.x,bogieComputed.x,0.5);
        EXPECT_NEAR(bogie.y,bogieComputed.y,0.5);
    }

}

TEST(PoseTest, GlobalToLocal)
{

    Pose aircraft;
    //Point bogie = {100,100,100};
    aircraft.position = {0,0,0};
    aircraft.orientation = tf::yawToQuaternion(0.785398);

    {
        Point bogie = {3171.34,-4574.67,0};
        //RangeBearingStamped rb = {5566.41,4.53316,0};
        RangeBearingStamped rb;
        rb.bearing = 1.76278;
        rb.range = 5566.42;
        rb.timestamp = 0;

        RangeBearingStamped rangeBearingStamped = tf2::global2local(bogie,aircraft);
        std::cout<<"rangeBearingStamped.bearing = "<< rangeBearingStamped.bearing<<endl;
        std::cout<<"rangeBearingStamped.range = "<< rangeBearingStamped.range<<endl;
        std::cout<<"rangeBearingStamped.timestamp = "<< rangeBearingStamped.timestamp<<endl;

        EXPECT_NEAR(rb.bearing,rangeBearingStamped.bearing,0.5);
        EXPECT_NEAR(rb.range,rangeBearingStamped.range,0.5);
        EXPECT_NEAR(rb.timestamp,rangeBearingStamped.timestamp,0.5);
    }

    {
        Point bogie = {-3288.52,-2516.65,0};
        //RangeBearingStamped rb = {4141,3.0094,0};
        RangeBearingStamped rb;
        rb.bearing = 3.00941;
        rb.range = 4141;
        rb.timestamp = 0;
        RangeBearingStamped rangeBearingStamped = tf2::global2local(bogie,aircraft);
        EXPECT_NEAR(rb.bearing,rangeBearingStamped.bearing,0.5);
        EXPECT_NEAR(rb.range,rangeBearingStamped.range,0.5);
        EXPECT_NEAR(rb.timestamp,rangeBearingStamped.timestamp,0.5);

    }

}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
