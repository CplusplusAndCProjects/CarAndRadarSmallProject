#include "radar_interface.h"

Radar_Interface::Radar_Interface()
{
    model_ = "Unknown";
}

string Radar_Interface::getModel(){
    return model_;
}

