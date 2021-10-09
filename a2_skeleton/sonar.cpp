#include "sonar.h"



Sonar::Sonar()

{
    model_= Sonar_Model;
    max_distance_=Sonar_maxrange;
    min_distance_=Sonar_minrange;
    field_of_view_ =  Sonar_FOV;
    offset_ = 0; //defaut offset
    sensingMethod_ = ranger::CONE;
}

int Sonar::getNumberOfSample()
{
    num_of_samples_ = 1;
    return num_of_samples_;
}

Sonar::~Sonar()
{
    
}







