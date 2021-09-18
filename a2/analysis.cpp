#include "analysis.h"

#include <iostream> // Only here for showing the code is working
#include <thread>

Analysis::Analysis(std::shared_ptr<Radar> radarPtr):
    radarPtr_(radarPtr)
{

}

//! @todo
//! TASK 1 and 2 - Same implementation, just being called twice Refer to README.md and the Header file for full description
void Analysis::computeScanningSpeed(unsigned int samples, double& scanningSpeed){
  auto start = std::chrono::steady_clock::now();
  std::cout<<"Number sample will scann: "<< samples <<std::endl;
  std::cout<<"With max disstance is:  "<< radarPtr_->getMaxDistance() <<std::endl;
  for(int i =0; i<=samples;i++){
    radarPtr_->getData();
    auto endTemp = std::chrono::steady_clock::now();

    std::chrono::duration<double> diff1 = endTemp - start;

  }

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> diff = end - start;
  double totalTime = diff.count();
  std::cout<<"total time :"<< totalTime<<std::endl;
  scanningSpeed = totalTime / samples;
  return;
}
