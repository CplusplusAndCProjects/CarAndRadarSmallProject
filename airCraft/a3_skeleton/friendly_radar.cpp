#include "friendly_radar.h"

Friendly_Radar::Friendly_Radar(shared_ptr<Simulator> &sim){
    model_ = "friendly Radar";
    sim_ = sim;
}

void Friendly_Radar::setSimulator(shared_ptr<Simulator> &sim){
    sim_ = sim;
}

vector<RangeBearingStamped> Friendly_Radar::getRangeBearingData(){
    friendly_data_ = sim_->rangeBearingToBogiesFromFriendly();
    return friendly_data_;
}

vector<RangeVelocityStamped> Friendly_Radar::getRangeVelocityData(){
  
}





