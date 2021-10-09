#ifndef LASER_H
#define LASER_H

#include "ranger.h"

const ranger::SensingMethod Laser_SensingMethod = ranger::POINT;

class Laser: public Ranger
{
public:
  //Default constructor - should set all sensor attributes to a default value
  Laser();
  Laser(string mModel, double mMaxDistance,double  mMinDistance,int mFieldOffView, int mOffsetOfSensor,ranger::SensingMethod mSensingMethod, int mAngularResolution)
{
};
  ~Laser(){};
  /**
   * @brief   The virtual function to be implemented inside the derived class
   * @return  The number of samples as an int
   */
int getNumberOfSample();

};

#endif // LASER_H
