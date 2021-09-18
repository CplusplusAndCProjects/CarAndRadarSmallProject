#include "analysis.h"
#include"iostream"
using std::vector;
using std::pair;

std::vector<unsigned int> runningTime;

//! @todo
//! TASK 1 - We need to store cars in the Analysis class, how to do this?
Analysis::Analysis(std::vector<Car*> cars) :
    cars_(cars),raceDisplay_(nullptr)
{
    raceDisplay1 = raceDisplay_;

}

Analysis::Analysis(std::vector<Car*> cars,std::shared_ptr<DisplayRace> raceDisplay) :
    cars_(cars),raceDisplay_(raceDisplay)
{
    raceDisplay1 = raceDisplay_;
}

//! @todo
//! TASK 1 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::sortByOdometry(){

    cout<<"sortByOdometry"<<endl;

    std::vector<unsigned int> order(cars_.size(),0);//Creating a vector, same size as cars with all zeros
    //initial oder array with sort default
    for (int i = 0; i < order.size(); i++)
    {
        order[i] = i;
        /* code */
    }
    cout<<"number of Cars: "<< cars_.size()<<endl;
    std::vector<Car*> orderTemp = cars_;
    int oderTemp = 0;

    Car *carTemp = cars_.at(0);
    //carTemp = cars_[0];
    for (int i = 0; i < cars_.size()-1; i++)
    {   

        for (int j = i+1; j<cars_.size(); j++)
        {

            /* code */
            if(orderTemp[i]->getOdometry()>orderTemp[j]->getOdometry())
                {

                        oderTemp = order[i];
                        carTemp = orderTemp.at(i);

                        order[i] = order[j];
                        orderTemp[i] = orderTemp.at(j);

                        order[j] = oderTemp;
                        orderTemp[j] = carTemp;
                }
        }
    }

    return order;
}

//! @todo
//! TASK 2 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::dragRace(double distance){
    cout<<"dragRace"<<endl;


    std::vector<unsigned int> order(cars_.size(),0);

    std::vector<unsigned int> runningTime(cars_.size(),0);

    std::vector<Car*> orderTemp = cars_;
    int oderTemp = 0, runningTimeTemp = 0;


    //initial oder array with sort default
    for (int i = 0; i < order.size(); i++)
    {
        order[i] = i;
        /* code */
    }

    for (int i = 0; i < cars_.size(); i++)
    {
        runningTime[i] = (RacingWithDistance(cars_[i],distance));

    }
    for (int i = 0; i < runningTime.size(); i++)
    {
        cout<<"runningTime["<<i<<"]:"<<runningTime[i]<<endl;

    }
    for (int i = 0; i < cars_.size()-1; i++)
    {
        for (int j = i+1; j<cars_.size(); j++)
        {
            /* code */
        if(runningTime[i]>runningTime[j])
            {

                    oderTemp = order[i];
                    runningTimeTemp = runningTime.at(i);

                    order[i] = order[j];
                    runningTime[i] = runningTime.at(j);

                    order[j] = oderTemp;
                    runningTime[j] = runningTimeTemp;
            }
        }
    }

    return order;
}


//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
void Analysis::stopAllCars(){

    cout<<"stopAllCars"<<endl;

    for (int i = 0; i < cars_.size(); i++)
    {
        /* code */
        while (cars_[i]->getCurrentSpeed()>0)
        {
            /* code */
            cars_[i]->decelerate();
            usleep(100);
        }
        std::cout<<" Car ["<<i<<"] was stop, car speed: "<< cars_[i]->getCurrentSpeed()<<endl;
    }
}

//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::zeroTopZeroRace(){

    cout<<"zeroTopZeroRace"<<endl;
    std::vector<unsigned int> order(cars_.size(),0);//Creating a vector, same size as cars with all zeros
    
    std::vector<unsigned int> runningTime1(cars_.size(),0);
    runningTime = runningTime1;
    std::vector<Car*> orderTemp = cars_;
    int oderTemp = 0, runningTimeTemp = 0;
    thread threads[cars_.size()];
    int rc;
    //initial oder array with sort default
    for (int i = 0; i < order.size(); i++)
    {
        order[i] = i;
        /* code */
    }

    for (int i = 0; i < cars_.size(); i++)
    {

        threads[i] = thread(RacingWithSpeed, cars_[i]); 
        //th.join(); 
        //runningTime.push_back(RacingWithSpeed(cars_[i]));      // Call function and assign return value.
    }

    for (int i = 0; i < cars_.size(); i++)
    {

        //threads[i] = thread(RacingWithSpeed, cars_[i]); 
        threads[i].join(); 
        //runningTime.push_back(RacingWithSpeed(cars_[i]));      // Call function and assign return value.
    }
    for (int i = 0; i < cars_.size()-1; i++)
    {
        for (int j = i+1; j<cars_.size(); j++)
        {
            /* code */
        if(runningTime[i]>runningTime[j])
            {

                    oderTemp = order[i];
                    runningTimeTemp = runningTime.at(i);

                    order[i] = order[j];
                    runningTime[i] = runningTime.at(j);

                    order[j] = oderTemp;
                    runningTime[j] = runningTimeTemp;
            }
        }
    }
    
    return order;
}

// Demo code
void Analysis::demoRace(){


    //This is an example of how to draw 3 cars moving
    // accelerate 300 times
    unsigned int count=0;

    while(count < 300){
        for(auto car : cars_){
          car->accelerate();
        }
        if(raceDisplay_!=nullptr){
            raceDisplay_->updateDisplay();
        }
        count++;
    }

    // decelerate 600 times
    count =0;
    while(count < 600){
        for(auto car : cars_){
          car->decelerate();
        }
        if(raceDisplay_!=nullptr){
            raceDisplay_->updateDisplay();
        }
        count++;
    }

}

int Analysis::RacingWithDistance(Car *car, int distance)
{

    cout<<"RacingWithDistance"<<endl;

    int time = 0;
    double odoMetryBegin = car->getOdometry();
    while (distance + odoMetryBegin > car->getOdometry())
    {
        /* code */
        car->accelerate();
        distance = distance - time*car->getCurrentSpeed();
        time++;
        usleep(100);
        if(raceDisplay_!=nullptr){
            raceDisplay_->updateDisplay();
        }
    }
    
    return time;
}
int Analysis::RacingWithSpeed(Car *car)
{
    cout<<"RacingWithSpeed"<<endl;

    cout<<"accelerate"<<endl;

    int time = 0;
    while (car->getTopSpeed() > car->getCurrentSpeed())
    {
        /* code */
        time++;
        car->accelerate();
        if(raceDisplay1!=nullptr){
            raceDisplay1->updateDisplay();
        }        
        usleep(100);
    }
    cout<<"decelerate"<<endl;    
    while (0 < car->getCurrentSpeed())
    {
        /* code */
        time++;
        car->decelerate();
        if(raceDisplay1!=nullptr){
            raceDisplay1->updateDisplay();
        }           
        usleep(100);
    }

    runningTime.push_back(time);
    
}