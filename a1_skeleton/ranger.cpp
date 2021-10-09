#include "ranger.h"
#include <thread>
#include <unistd.h>
// Constructor for Ranger class that initialize model, min and max range to default settings
Ranger::Ranger(): mModel("Unknown"), mMinDistance(0.0), mMaxDistance(0.0)
{

}

//<! The function that generate the raw data
std::vector<double> Ranger::generateData()
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::normal_distribution<double> distribution (MEAN,STAND_DEVIATION);           // Set up the random generator
    vector<double> data(getNumberOfSample());
    for(vector<double>::iterator it = data.begin(); it!=data.end(); it++)
    {
        while(1)
        {
            //While loop to wait for the in range value to be generated
            double temp = distribution(generator);
            if((temp>=mMinDistance)&&(temp<=mMaxDistance))
            {
                *it = temp;
                break;
            }
        }
    }
    mData = data;
    return mData;
}

//<! The funtion to get the angular resolution
unsigned int Ranger::getAngularResolution()
{
    return mAngularResolution;
}

//<! The funtion to get the angular offset
int Ranger::getAngularOffset()
{
    return mOffsetOfSensor;
}
ranger::SensorPose Ranger::getSensorPose(void){
    return mSensorPose;
}

//<! The funtion to get the FOV
unsigned int Ranger::getFieldOfView()
{
    return mFieldOffView;
}

//<! The funtion to calculate the sample for each ranger
int Ranger::getNumberOfSample()
{
    int num_of_samples = 1 + 2 * ((getFieldOfView()/2)/getAngularResolution());
    mNumOfSamples = num_of_samples;
    return mNumOfSamples;
}

//<! The funtion to check the valid input off all the rangers
bool Ranger::setAngularResolution(unsigned int resolution)
{
    if((mAngularResolution=resolution) && ((mAngularResolution==SMALL_ANGULAR)||(mAngularResolution==BIG_ANGULAR)))
        return true;
    else
    {
        mAngularResolution = DEFAULT_ANGULAR_RESOLUTION;
        return false;
    }
}

//<! The funtion to check the valid offset input off all the rangers
bool Ranger::setAngularOffset(int offset)
{
    if((mOffsetOfSensor = offset)&&(mOffsetOfSensor>=MIN_OFFSET)&&(mOffsetOfSensor<=MAX_OFFSET))
        return true;
    else
    {
        mOffsetOfSensor=DEFAULT_ANGULAR_OFFSET;
        return false;
    }
}
bool Ranger:: setSensorPose(ranger::SensorPose pose){
    if((mSensorPose.theta = pose.theta)&&(mSensorPose.theta>=MIN_OFFSET)&&(mSensorPose.theta<=MAX_OFFSET))
        {
            mSensorPose = pose;
            setAngularOffset(mSensorPose.theta);
        return true;}
    else
    {
        mSensorPose = pose;
        mSensorPose.theta = DEFAULT_ANGULAR_OFFSET;
        setAngularOffset(mSensorPose.theta);
        return false;
    }
}

//<! The funtion to set the FOV
bool Ranger::setFieldOfView(unsigned int fov)
{
    mFieldOffView=fov;
    return true;
}

//<! The funtion to get the model
std::string Ranger::getModel()
{
    return mModel;
}

//<! The funtion to get the ranger max range
double Ranger::getMaxRange(void)
{
    return mMaxDistance;
}

//<! The funtion to get the ranger min range
double Ranger::getMinRange(void)
{
    return mMinDistance;
}

//<! Delay function
void Ranger::delay()
{
   //std::this_thread::sleep_for(chrono::milliseconds(SAMPLING_TIME));     // Sleep function used for delay
   //_sleep(1000);
    sleep(1);
}

//<! The funtion to get the ranger sensing range
ranger::SensingMethod Ranger::getSensingMethod(void)
{
    return ranger::SensingMethod();
}


