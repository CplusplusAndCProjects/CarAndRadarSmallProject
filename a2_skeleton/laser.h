#ifndef LASER_H
#define LASER_H

#include "ranger.h"

const std::string Laser_Model = "SICK-XL"; /*!< Default model name of the laser sensor*/
const ranger::SensingMethod Laser_SensingMethod = ranger::POINT;
const double Laser_minrange = 0.2;  /*!< Default data min range of the laser sensor*/
const int Angular_reslution_10 = 10;  /*!< Default angular resolution:10 degree of the laser sensor*/
const int Angular_reslution_30 = 30;  /*!< Default angular resolution:30 degree of the laser sensor*/
const int Laser_FOV = 180;  /*!< Default field of view of the laser sensor*/
const int Laser_maxrange = 8;  /*!< Default data max range of the laser sensor*/

/*!
 * \brief   The Radar class derived from the base class Ranger
 * \author  Trong Duy Bui
 * \date    August25, 2020
 */
class Laser: public Ranger
{
public:
  //Default constructor should set all sensor attributes to a default value
    Laser();
    ~Laser();
    /**
     * @brief   The virtual function to be implemented inside the derived class
     * @return  The number of samples as an int
     */
    int getNumberOfSample();

private:


};

#endif // LASER_H
