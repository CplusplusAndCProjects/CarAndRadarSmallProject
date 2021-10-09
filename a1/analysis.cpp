#include "analysis.h"
#include"iostream"
using std::vector;
using std::pair;

std::vector<unsigned int> runningTimeStatic;
std::vector<unsigned int> runningDistanceStatic;

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
    std::vector<Car*> carOrderTemp = cars_;
    int oderTemp = 0;

    Car *carTemp = cars_.at(0);
    //carTemp = cars_[0];
    for (int i = 0; i < carOrderTemp.size()-1; i++)
    {   

        for (int j = i+1; j<carOrderTemp.size(); j++)
        {

            /* code */
            if(carOrderTemp[i]->getOdometry()>carOrderTemp[j]->getOdometry())
                {

                        //oderTemp = order[i];
                        carTemp = carOrderTemp[i];

                        //order[i] = order[j];
                        carOrderTemp[i] = carOrderTemp[j];

                        //order[j] = oderTemp;
                        carOrderTemp[j] = carTemp;
                }
        }
    }
    
    for (int i = 0; i < cars_.size(); i++)
    {
        for (int j = 0; j<carOrderTemp.size(); j++)
        {
            /* code */
            if(cars_[i]->getOdometry() == carOrderTemp[j]->getOdometry())
            {
                order[i] = j;
                break;
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
    std::vector<unsigned int> runningTimeTemps(cars_.size(),0);
    //std::vector<unsigned int> runningTime(cars_.size(),0);
    runningDistanceStatic = runningTimeTemps;
    //carsChange = cars_;
    //std::vector<Car*> orderTemp = cars_;
    Car *carTemp = cars_.at(0);
    int oderTemp = 0, runningTimeTemp = 0;
    thread threads[cars_.size()];

    //initial oder array with sort default
    for (int i = 0; i < order.size(); i++)
    {
        order[i] = i;
        /* code */
    }

    // for (int i = 0; i < cars_.size(); i++)
    // {
    //     runningTime[i] = (RacingWithDistance(cars_[i],distance, i));

    // }

        for (int i = 0; i < cars_.size(); i++)
    {

        threads[i] = thread(RacingWithDistance, cars_[i],distance, i); 
        //th.join(); 
        //runningTime.push_back(RacingWithSpeed(cars_[i]));      // Call function and assign return value.
    }

    for (int i = 0; i < cars_.size(); i++)
    {

        //threads[i] = thread(RacingWithSpeed, cars_[i]); 
        threads[i].join(); 
        //runningTime.push_back(RacingWithSpeed(cars_[i]));      // Call function and assign return value.
    }

    int a =0;
    while (a == 0)
    {
        int count =0;
        for (int i = 0; i < runningDistanceStatic.size(); i++)
        {   
            if(runningDistanceStatic[i]> 0)
                count ++;
            
        }
        if(count >= runningDistanceStatic.size()-1)
            a = 1;
        else
            count = 0;
        
        usleep(100);

    }
    runningTimeTemps = runningDistanceStatic;

    for (int i = 0; i < runningDistanceStatic.size(); i++)
    {
        cout<<"Time running of car["<<i<<"], Carname:"<<cars_[i]->getMake()<<" is: "<< runningDistanceStatic[i]<<endl;


    }
    for (int i = 0; i < runningTimeTemps.size()-1; i++)
    {
        for (int j = i+1; j<runningTimeTemps.size(); j++)
        {
            /* code */
        if(runningTimeTemps[i]>runningTimeTemps[j])
            {

                    //oderTemp = order[i];
                    runningTimeTemp = runningTimeTemps[i];
                    //carTemp = cars_[i];
                    

                    //order[i] = order[j];
                    runningTimeTemps[i] = runningTimeTemps[j];
                    //cars_[i]= cars_[j];

                    //order[j] = oderTemp;
                    runningTimeTemps[j] = runningTimeTemp;
                    //cars_[j]= carTemp;
                    
            }
        }
    }
    for (int i = 0; i < runningDistanceStatic.size(); i++)
    {
        for (int j = 0; j<runningTimeTemps.size(); j++)
        {
            /* code */
            if(runningDistanceStatic[i] == runningTimeTemps[j])
            {
                order[i] = j;
                break;
            }
        }
    }


    return order;
}


//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
void Analysis::stopAllCars(){

    thread threads[cars_.size()];

    for (int i = 0; i < cars_.size(); i++)
    {

        threads[i] = thread(StopCar, cars_[i],i); 
        //th.join(); 
        //runningTime.push_back(RacingWithSpeed(cars_[i]));      // Call function and assign return value.
    }

    for (int i = 0; i < cars_.size(); i++)
    {

        //threads[i] = thread(RacingWithSpeed, cars_[i]); 
        threads[i].join(); 
        //runningTime.push_back(RacingWithSpeed(cars_[i]));      // Call function and assign return value.
    }

    for (int i = 0; i < cars_.size(); i++)
    {
        cout<<"stop Cars name: "<<cars_[i]->getMake()<<endl;
        /* code */
        while (cars_[i]->getCurrentSpeed()>0)
        {
            /* code */
            cars_[i]->decelerate();
            if(raceDisplay_!=nullptr){
                raceDisplay_->updateDisplay();
            }
            usleep(20);
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
    runningTimeStatic = runningTime1;
    //std::vector<Car*> orderTemp = cars_;
    int oderTemp = 0, runningTimeTemp = 0;
    Car *carTemp = cars_.at(0);
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

        threads[i] = thread(RacingWithSpeed, cars_[i], i); 
        //th.join(); 
        //runningTime.push_back(RacingWithSpeed(cars_[i]));      // Call function and assign return value.
    }

    for (int i = 0; i < cars_.size(); i++)
    {

        //threads[i] = thread(RacingWithSpeed, cars_[i]); 
        threads[i].join(); 
        //runningTime.push_back(RacingWithSpeed(cars_[i]));      // Call function and assign return value.
    }

    int a =0;
    while (a == 0)
    {
        int count =0;
        for (int i = 0; i < runningTimeStatic.size(); i++)
        {   
            if(runningTimeStatic[i]> 0)
                count ++;
            
        }
        if(count >= runningTimeStatic.size()-1)
            a = 1;
        else
            count = 0;
        
        usleep(100);

    }
    
    std::vector<unsigned int> runningTimeTemps(cars_.size(),0);
    runningTimeTemps = runningTimeStatic;

    for (size_t i = 0; i < runningTimeStatic.size(); i++)
    {
        /* code */
        runningTimeTemps[i]= (runningTimeStatic.at(i));
    }
    cout<<"Size of runningTimeTemps "<<runningTimeTemps.size()<<endl;

    for (int i = 0; i < runningTimeTemps.size(); i++)
    {
        cout<<"Time running of car["<<i<<"], car name: "<<cars_[i]->getMake()<<" is: "<< runningTimeTemps[i]<<", init order is:"<<order[i]<<endl<<endl;
    }

    for (int i = 0; i < runningTimeTemps.size()-1; i++)
    {
        for (int j = i+1; j<runningTimeTemps.size(); j++)
        {
            /* code */
            if(runningTimeTemps[i]>runningTimeTemps[j])
            {
                    //carTemp = cars_[i];
                    //oderTemp = order[i];
                    runningTimeTemp = runningTimeTemps[i];

                    //cars_[i] = cars_[j];
                    //order[i] = order[j];
                    runningTimeTemps[i] = runningTimeTemps[j];

                    //cars_[j] = carTemp;
                    //order[j] = oderTemp;
                    runningTimeTemps[j] = runningTimeTemp;
            }
        }
    }
    for (int i = 0; i < runningTimeStatic.size(); i++)
    {
        for (int j = 0; j<runningTimeTemps.size(); j++)
        {
            /* code */
            if(runningTimeStatic[i] == runningTimeTemps[j])
            {
                order[i] = j;
                break;
            }
        }
    }

    for (int i = 0; i < order.size(); i++)
    {
        cout<<"Order after sort of car ["<<i<<"], car name: "<<cars_[i]->getMake()<<" is: "<< order[i]<<", runtime is: "<<runningTimeStatic[i]<<endl;
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
int Analysis::StopCar(Car *car, int index){

        cout<<"stop Cars name: "<<car->getMake()<<endl;
        /* code */
        while (car->getCurrentSpeed()>0)
        {
            /* code */
            car->decelerate();
            usleep(20);
            if(raceDisplay1!=nullptr){
                raceDisplay1->updateDisplay();
            }
        }
        std::cout<<" Car ["<<index<<"] was stop, car speed: "<< car->getCurrentSpeed()<<endl;

}
int Analysis::RacingWithDistance(Car *car, int distance, int index)
{

    cout<<"Racing With Distance"<<endl;
    auto start = std::chrono::steady_clock::now();

    double odoMetryBegin = car->getOdometry();
    while (distance + odoMetryBegin > car->getOdometry())
    {
        /* code */
        car->accelerate();
        //time++;
        usleep(10);
        if(raceDisplay1!=nullptr){
            raceDisplay1->updateDisplay();
        }
    }
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = end - start;
    runningDistanceStatic[index] = diff.count();
    return diff.count();
}
int Analysis::RacingWithSpeed(Car *car, int index)

{

    cout<<"RacingWithSpeed"<<endl;

    cout<<"accelerate"<<endl;
    auto start = std::chrono::steady_clock::now();

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

    auto end = std::chrono::steady_clock::now();
    
    std::chrono::duration<double> diff = end - start;

    runningTimeStatic[index] = (diff.count());
    //cout<<"Time running car["<<index<<"], Carname:"<<car->getMake()<<" is: "<< diff.count()<<endl;
    
}