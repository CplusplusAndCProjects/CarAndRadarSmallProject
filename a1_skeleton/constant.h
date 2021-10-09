#ifndef CONSTANT_H
#define CONSTANT_H
#include<string>
using namespace std;
// contant for ranger base class
const double MEAN = 5.0;              //!< Mean value for normal distribution
const double STAND_DEVIATION = 6.0;           //!< Standard deviation for normal distribution
const int MIN_OFFSET = -350;           //!< Minimum boundary for ranger offset
const int MAX_OFFSET = 350;            //!< Maximum boundary for ranger offset
const int DEFAULT_ANGULAR_RESOLUTION = 10;    //!< Default value for angular resolution
const int DEFAULT_ANGULAR_OFFSET = 0;         //!< Default value for offset
const int SAMPLING_TIME = 1000;       //!< Anticipated sampling time
const int SMALL_ANGULAR = 10;         //!< Default angular resolution:10 degree of the laser sensor
const int BIG_ANGULAR = 30;           //!< Default angular resolution:30 degree of the laser sensor


// contant for laser class
const string LASER_MODEL_SICK_XL = "SICK-XL"; /*!< Default model name of the laser sensor*/
const double LASER_MIN_RANGE = 0.2;  /*!< Default data min range of the laser sensor*/
const int LASER_ANGULAR_RESOLUTION_10 = 10;  /*!< Default angular resolution:10 degree of the laser sensor*/
const int LASER_ANGULAR_RESOLUTION_30 = 30;  /*!< Default angular resolution:30 degree of the laser sensor*/
const int LASER_FIELD_OF_VIEW = 180;  /*!< Default field of view of the laser sensor*/
const double LASER_MAX_RANGE = 8.0;  /*!< Default data max range of the laser sensor*/

// contant for sonar base class
const string SONAR_MODEL_SN_001 = "SN-001"; /*!< Default model name of the sonar sensor*/
const int SONAR_FIELD_OF_VIEW = 20; /*!< Default field of view of the sonar sensor*/
const int SONAR_ANGULAR_RESOLUTION = 0;  /*!< Default angular resolution:0 degree of the sonar sensor*/
const int SONAR_MAX_RANGE = 10.0; /*!< Default data max range of the sonar sensor*/
const double SONAR_MIN_RANGE = 0.2; /*!< Default data min range of the sonar sensor*/

// contant for ranger funsion  class
const double PI = 2*acos(0);
const int FUSION_FIELD_OF_VIEW = 20;
const int FUSION_RANGE = 180;
const int FUSION_LASER_FIELD_OF_VIEW = LASER_FIELD_OF_VIEW;
const int FUSION_SONAR_FIELD_OF_VIEW = SONAR_FIELD_OF_VIEW;
#endif // SONAR_H
