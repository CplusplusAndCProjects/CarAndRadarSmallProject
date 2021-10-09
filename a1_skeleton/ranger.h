#ifndef RANGER_H
#define RANGER_H

#include <string>
#include <chrono>
#include <random>
#include <thread>

#include "rangerinterface.h"
#include "constant.h"
using namespace std;

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
    ranger::SensorPose getSensorPose(void);

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
    bool setSensorPose(ranger::SensorPose pose);
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



    public:
    string mModel;                        //!< Attribute to hold the model
    vector<double> mData;                 //!< Attribute to hold the generated readings
    unsigned int mAngularResolution;     //!< Attribute to hold the angular resolution
    int mOffsetOfSensor;                          //!< Attribute to hold the offset of sensor
    unsigned int mFieldOffView;          //!< Attribute to hold the field of view
    int mNumOfSamples;                  //!< Attribute to hold the number of samples to generate
    double mMaxDistance;                 //!< Attribute to hold the maximum range
    double mMinDistance;                 //!< Attribute to hold the minimum distance
    ranger::SensingMethod mSensingMethod; //!< Attribute to hold the sensing method
    ranger::SensorPose mSensorPose;

};

#endif // RANGER_H
