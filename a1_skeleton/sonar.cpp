#include "sonar.h"

Sonar::Sonar()

{
    mModel = (SONAR_MODEL_SN_001);
    mMaxDistance = (SONAR_MAX_RANGE);
    mMinDistance = (SONAR_MIN_RANGE);
    mFieldOffView = (SONAR_FIELD_OF_VIEW);
    mOffsetOfSensor = (0);
    mSensingMethod = (ranger::CONE);
}

int Sonar::getNumberOfSample()
{
    mNumOfSamples = 1;
    return mNumOfSamples;
}

Sonar::~Sonar()
{

}

