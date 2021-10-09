#include "laser.h"
#include <random>
#include <chrono>

Laser::Laser()
{
    model_ = Laser_Model;
    max_distance_= Laser_maxrange;
    min_distance_= Laser_minrange;
    field_of_view_ = Laser_FOV;
    angular_resolution_ =  Angular_reslution_10;
    offset_ = 0;
    sensingMethod_ = ranger::POINT;
    
}

int Laser::getNumberOfSample()
{
    int num_of_samples = 1 + 2 * ((getFieldOfView()/2)/getAngularResolution());
    num_of_samples_ = num_of_samples;
    return num_of_samples_;
}

Laser::~Laser()
{
    
}





