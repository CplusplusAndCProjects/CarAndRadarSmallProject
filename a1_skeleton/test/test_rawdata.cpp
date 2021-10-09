#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "laser.h"
#include "rangerfusion.h"
#include "ranger.h"
#include "sonar.h"
#include "cell.h"

using namespace std;

//Test that multiple sensors produce a larger rawDataArray
TEST (MultipleSensorsTest, LaserAndSonarSimple) {
  //Sensors:
  //  - (1) Laser {fov=180, angular_res=30, pose=0,0,0}
  //  - (1) Sonar {fov=60, angular_res=20, pose=0,0,0}
  //
  // Produces two vectors, the first with 7 measurements and the second with
  // 1.
  Sonar s;
  s.setSensorPose({0,0,0});

  Laser l;
  l.setAngularResolution(30);
  l.setSensorPose({0,0,0});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0,0.5);
  cells.at(0)->setSide(0.2);
  std::vector<RangerInterface *> sensors = { &l, &s };
  RangerFusion rf(sensors);
  rf.setCells(cells);
  rf.grabAndFuseData();

  auto rawdata = rf.getRawRangeData();

  ASSERT_EQ(2, rawdata.size());
  ASSERT_EQ(7, rawdata[0].size());
  ASSERT_EQ(1, rawdata[1].size());
}

TEST (MultipleSensorsTest, LaserAndSonarWithOffset) {
  //Sensors:
  //  - (1) Laser {fov=180, angular_res=10, pose=0,0,0}
  //  - (1) Sonar {fov=60, angular_res=20, pose=0,0,90}
  //
  // Offset should not alter the raw data produced.
  // Produces two vectors, the first with 19 measurements and the second with
  // 3.
  Sonar s;
  s.setSensorPose({0,0,90*M_PI});

  Laser l;
  l.setAngularResolution(10);
  l.setSensorPose({0,0,0});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0,0.5);

  std::vector<RangerInterface *> sensors = { &l, &s };
  RangerFusion rf(sensors);
  rf.setCells(cells);
  rf.grabAndFuseData();

  auto rawdata = rf.getRawRangeData();

  ASSERT_EQ(2, rawdata.size());
  ASSERT_EQ(19, rawdata[0].size());
  ASSERT_EQ(1, rawdata[1].size());
}

//Test that multiple sensors produce a larger rawDataArray with least of two sonar sensor
TEST (MultipleSensorsTest, LaserAndMultiSonarSimple) {
  //Sensors:
  //  - (1) Laser {fov=180, angular_res=30, pose=0,0,0}
  //  - (1) Sonar {fov=60, angular_res=20, pose=0,0,0}
  //
  // Produces two vectors, the first with 7 measurements and the second with
  // 1.
  Sonar s1,s2;
  s1.setSensorPose({0,0,0});

  s2.setSensorPose({0,0,5});

  Laser l;
  l.setAngularResolution(30);
  l.setSensorPose({0,0,0});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0,0.5);

  std::vector<RangerInterface *> sensors = { &l, &s1, &s2 };
  RangerFusion rf(sensors);
  rf.setCells(cells);
  cells.at(0)->setSide(0.2);
  rf.grabAndFuseData();

  auto rawdata = rf.getRawRangeData();
  cout<<"rawdata size = "<<rawdata.size();
  for(int i = 0; i<rawdata.size(); i++){

      cout<<"size of mData ["<<i<<"] ="<< rawdata[i].size(); 

  }
  ASSERT_EQ(3, rawdata.size());
  ASSERT_EQ(7, rawdata[0].size());
  ASSERT_EQ(1, rawdata[1].size());
  ASSERT_EQ(1, rawdata[2].size());
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
