#include "ranger.h"
#include <unistd.h>

using namespace std;

// Constructor for Ranger class that initialize model, min and max range to default settings
Ranger::Ranger():
    model_("Unknown"),min_distance_(0.0),max_distance_(0.0)
{

}

//<! The function that generate the raw data
std::vector<double> Ranger::generateData()
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::normal_distribution<double> distribution (mean,std_dev);           // Set up the random generator
    vector<double> data(getNumberOfSample());
    for(vector<double>::iterator it = data.begin(); it!=data.end();it++)
    {
        while(1)
        {                                                           //While loop to wait for the in range value to be generated
            double temp = distribution(generator);
            if((temp>=min_distance_)&&(temp<=max_distance_))
            {
                *it = temp;
                break;
            }
        }
    }
    data_ = data;
    return data_;
}

//<! The funtion to get the angular resolution
unsigned int Ranger::getAngularResolution()
{
    return angular_resolution_;
}

//<! The funtion to get the angular offset
int Ranger::getAngularOffset()
{
    return offset_;
}

//<! The funtion to get the FOV
unsigned int Ranger::getFieldOfView()
{
    return field_of_view_;
}

//<! The funtion to calculate the sample for each ranger
int Ranger::getNumberOfSample()
{
    int num_of_samples = 1 + 2 * ((getFieldOfView()/2)/getAngularResolution());
    num_of_samples_ = num_of_samples;
    return num_of_samples_;
}

//<! The funtion to check the valid input off all the rangers
bool Ranger::setAngularResolution(unsigned int resolution)
{
    if((angular_resolution_=resolution)&&((angular_resolution_==small_angular)||(angular_resolution_==big_angular)))
    return true;
    else
    {
       angular_resolution_ = default_Angularresolution;
       return false;
    }
}

//<! The funtion to check the valid offset input off all the rangers
bool Ranger::setAngularOffset(int offset)
{
    if((offset_=offset)&&(offset_>=min_offset)&&(offset_<=max_offset))
        return true;
    else
    {
        offset_=default_Angularoffset;
        return false;
    }
}

//<! The funtion to set the FOV
bool Ranger::setFieldOfView(unsigned int fov)
{
    field_of_view_=fov;
    return true;
}

//<! The funtion to get the model
std::string Ranger::getModel()
{
    return model_;
}

//<! The funtion to get the ranger max range
double Ranger::getMaxRange(void)
{
    return max_distance_;
}

//<! The funtion to get the ranger min range
double Ranger::getMinRange(void)
{
    return min_distance_;
}

//<! Delay function
void Ranger::delay()
{
    //std::this_thread::sleep_for(std::chrono::milliseconds(sampling_time));     // Sleep function used for delay
        sleep(1);

}

//<! The funtion to get the ranger sensing range
ranger::SensingMethod Ranger::getSensingMethod(void)
{
    return ranger::SensingMethod();
}


