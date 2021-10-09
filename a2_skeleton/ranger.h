#ifndef RANGER_H
#define RANGER_H

#include <string>
#include <chrono>
#include <random>
#include <thread>

#include "rangerinterface.h"

// const double mean = 4.0;              //!< Mean value for normal distribution
// const double std_dev = 5.0;           //!< Standard deviation for normal distribution
// const int min_offset = -350;           //!< Minimum boundary for ranger offset
// const int max_offset = 350;            //!< Maximum boundary for ranger offset
// const int default_Angularresolution = 10;    //!< Default value for angular resolution
// const int default_Angularoffset = 0;         //!< Default value for offset
// const int sampling_time = 1000;       //!< Anticipated sampling time
// const int small_angular = 10;         //!< Default angular resolution:10 degree of the laser sensor
// const int big_angular = 30;           //!< Default angular resolution:30 degree of the laser sensor
const double mean = 5.0;              //!< Mean value for normal distribution
const double std_dev = 6.0;           //!< Standard deviation for normal distribution
const int min_offset = -350;           //!< Minimum boundary for ranger offset
const int max_offset = 350;            //!< Maximum boundary for ranger offset
const int default_Angularresolution = 10;    //!< Default value for angular resolution
const int default_Angularoffset = 0;         //!< Default value for offset
const int sampling_time = 1000;       //!< Anticipated sampling time
const int small_angular = 10;         //!< Default angular resolution:10 degree of the laser sensor
const int big_angular = 30;           //!< Default angular resolution:30 degree of the laser sensor



using namespace std;

/*!
 * \brief   The base class for Laser and Radar that inherits from the RangerInterface.
 * \details
 * This class is inheritted from the RangerInterface class, serves as the base class for all sensors
 * \author  Trong Duy Bui
 * \date    August 25, 2020
 */

class Ranger: public RangerInterface
{
    public:
      //Default constructor should set all sensor attributes to a default value
    Ranger(); 

    /**
      Member function get Sensing Method
      * @brief  Generate sensing Method for the sensor
      * @return side of cell [m]
     */
    ranger::SensingMethod getSensingMethod(void);
   
    /**
     * @brief   Generate random values
     * @return  The vector of values
     */
    vector<double> generateData();

    /**
     * @brief
     * @return  The model as a string
     */
    string getModel();

    /**
     * @brief   Getter for angular resolution
     * @return  Angular resolution as an unsigned int
     */
    unsigned int getAngularResolution(void);

    /**
     * @brief   Getter for field of view
     * @return  Field of view as an unsigned int
     */
    unsigned int getFieldOfView(void);

    /**
     * @brief   Getter of current offset
     * @return  Offset as an unsigned int
     */
    int getAngularOffset(void);  

    /**
     * @brief   Virtual function getter for current number of samples
     * @return  Number of samples as an int
     */
    virtual int getNumberOfSample() = 0;

    /**
     * @brief   This function sets the angular resolution as of the parameter
     * @param   The desired angular resolution
     * @return  Whether the angular resolution is set successfully
     */
    bool setAngularResolution(unsigned int resolution);

    /**
     * @brief   This function sets the offset for the ranger
     * @param   The desired offset for the ranger
     * @return  Whether the offset is set successfully
     */
    bool setAngularOffset(int offset);

    /**
     * @brief   This functions set the field of view for the ranger,
     * @param   The desired field of view
     * @return  Whether the field of view is set successfully
     */
    bool setFieldOfView(unsigned int fov);

    /**
     * @brief   Delay to achieve the assumed sampling rate of 1Hz
     */
    void delay();

    /**
     * @brief   Getter for maximum distance
     * @return  Max range as a double
     */
    double getMaxRange(void);

    /**
     * @brief   Getter for minimum range
     * @return  Minimum range as a double
     */
    double getMinRange(void);

    protected:
    string model_;                        //!< Attribute to hold the model
    vector<double> data_;                 //!< Attribute to hold the generated readings
    unsigned int angular_resolution_;     //!< Attribute to hold the angular resolution
    int offset_;                          //!< Attribute to hold the offset
    unsigned int field_of_view_;          //!< Attribute to hold the field of view
    int num_of_samples_;                  //!< Attribute to hold the number of samples to generate
    double max_distance_;                 //!< Attribute to hold the maximum range
    double min_distance_;                 //!< Attribute to hold the minimum distance
    ranger::SensingMethod sensingMethod_; //!< Attribute to hold the sensing method


};

#endif // RANGER_H
