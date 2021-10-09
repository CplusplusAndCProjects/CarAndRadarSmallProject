#ifndef SONAR_H
#define SONAR_H

#include "ranger.h"

const ranger::SensingMethod Sonar_SensingMethod = ranger::CONE; /*!< Default sensing method of the sonar sensor*/

class Sonar: public Ranger
{
public:
  //Default constructor should set all sensor attributes to a default value
  Sonar();
  ~Sonar();
        /**
     * @brief   The virtual function to be implemented inside the derived class
     * @return  The number of samples as an int
     */
  int getNumberOfSample();
};

#endif // SONAR_H
