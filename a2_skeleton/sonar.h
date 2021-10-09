#ifndef SONARR_H
#define SONARR_H

#include <string>

#include "ranger.h"
// const std::string Sonar_Model = "SN-001"; /*!< Default model name of the sonar sensor*/
// const ranger::SensingMethod Sonar_SensingMethod = ranger::CONE; /*!< Default sensing method of the sonar sensor*/
// const int Sonar_FOV = 20; /*!< Default field of view of the sonar sensor*/
// const int Sonar_maxrange = 16.0; /*!< Default data max range of the sonar sensor*/
// const double Sonar_minrange = 0.2; /*!< Default data min range of the sonar sensor*/


const std::string Sonar_Model = "SN-001"; /*!< Default model name of the sonar sensor*/
const ranger::SensingMethod Sonar_SensingMethod = ranger::CONE; /*!< Default sensing method of the sonar sensor*/
const int Sonar_FOV = 20; /*!< Default field of view of the sonar sensor*/
const int Sonar_maxrange = 10.0; /*!< Default data max range of the sonar sensor*/
const double Sonar_minrange = 0.2; /*!< Default data min range of the sonar sensor*/

/*!
 * \brief   The Radar class derived from the base class Ranger
 * \author  Trong Duy Bui
 * \date    August25, 2020
 */
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
    
private:


};

#endif // SONAR_H
