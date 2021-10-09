#include"laser.h"
Laser::Laser()
{
mModel= (SONAR_MODEL_SN_001);
mMaxDistance = (LASER_MAX_RANGE);
mMinDistance = (LASER_MIN_RANGE);
mFieldOffView = (LASER_FIELD_OF_VIEW);
mOffsetOfSensor= (0);
mSensingMethod = (ranger::POINT);
mAngularResolution = (LASER_ANGULAR_RESOLUTION_10);
}
int Laser::getNumberOfSample()
{
    int num_of_samples = 1 + 2 * ((getFieldOfView()/2)/getAngularResolution());
    mNumOfSamples = num_of_samples;
    return mNumOfSamples;
}
