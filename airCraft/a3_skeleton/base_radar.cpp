#include "base_radar.h"

Base_Radar::Base_Radar(shared_ptr<Simulator> & sim){
    model_ = "Base Radar";
    sim_ = sim;
}

void Base_Radar::setSimulator(shared_ptr<Simulator> & sim){
    sim_ = sim;
}

vector<RangeVelocityStamped> Base_Radar::getRangeVelocityData(){
    base_data_ = sim_->rangeVelocityToBogiesFromBase();
    return base_data_;
}

vector<RangeBearingStamped> Base_Radar::getRangeBearingData(){


}



