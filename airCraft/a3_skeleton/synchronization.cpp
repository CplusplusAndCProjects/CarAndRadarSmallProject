#include "synchronization.h"

Synchronization::Synchronization(vector<Radar_Interface*> radars, shared_ptr<Simulator> & sim){
        radar_container_ = radars;
        sim_ = sim;
        for(int i = 0; i < radar_container_.size(); i++){
            //Get the position of each radar inside the vector
            if ((*radar_container_[i]).getModel() == "Base Radar") 
                base_pos_ = i;  
            else 
                friendly_pos_ = i;
        }
}

void Synchronization::RadarContainer(vector<Radar_Interface*> radars){
    radar_container_ = radars;
}

void Synchronization::BogieBaseRangeProducer(){
        vector<RangeVelocityStamped> BaseBogie_RangeVelocity_holder = radar_container_[base_pos_]->getRangeVelocityData();
        unique_lock<mutex> base_locker(baseradar_mutex_);    //Lock the mutex
        base_buffer_.push(BaseBogie_RangeVelocity_holder);           //Push the value to the buffer
        base_locker.unlock();
        base_cv_.notify_all();                          //Signal the appearance of data
}

void Synchronization::BogieFriendlyRangeProducer(){
        vector<RangeBearingStamped> BogieFriendly_RangeBearing_holder = radar_container_[friendly_pos_]->getRangeBearingData();
        unique_lock<mutex> friendly_locker(friendlyradar_mutex_);
        friendly_buffer_.push(BogieFriendly_RangeBearing_holder);
        friendly_locker.unlock();
        friendly_cv_.notify_all();

}

vector<RangeVelocityStamped> Synchronization::BogieBaseRangeConsumer(){              //get the distance from Bogie to Base
        unique_lock<mutex> base_locker(baseradar_mutex_);    //Lock the mutex
        while(base_buffer_.empty())                          //Check the content of the buffer
            base_cv_.wait(base_locker);                     //Wait for the signal
        unique_lock<mutex> sync_lock(sync_mutex_);          //Lock another mutex to ensure 10 friendly data coupled with 1 base data
        empty_ = false;                                  //Turn off the flag
        sync_lock.unlock();
        vector<RangeVelocityStamped> base_value_ = base_buffer_.back();//Get the newest data
        base_buffer_.pop();                                 //Clear the buffer
        base_locker.unlock();      
        return base_value_;
}

vector<RangeBearingStamped> Synchronization::BogieFriendlyRangeConsumer(){               //get the distance from Bogie to Friendly
        unique_lock<mutex> friendly_locker(friendlyradar_mutex_);
        unique_lock<mutex> sync_lock(sync_mutex_);
        while(empty_)                                                           //Check the flag
            friendly_cv_.wait(friendly_locker);                                     //Wait for signal from the producer
        sync_lock.unlock();
        empty_ = true;
        vector<RangeBearingStamped> friendly_value_ = friendly_buffer_.back(); //get the newest data from the radar
        friendly_buffer_.pop();
        friendly_locker.unlock();
        return friendly_value_;
}



double Synchronization::angleDifference(Pose pose, GlobalOrd o1){
        double alpha;
        double angle_difference;
        alpha = atan2(o1.y-pose.position.y, o1.x-pose.position.x);      //calculate the angle difference from -pi to pi
        if(alpha<0) alpha = alpha + 2*PI;                               //convert to 0 - PI resolusion
        angle_difference = alpha - pose.orientation;                    //Return the angle difference
        return angle_difference;
        
}

Speed Synchronization::Pure_Pursuit(const std::shared_ptr<Simulator> & sim){
        Pose friendly_pose = sim_->getFriendlyPose();           //Get the friendly pose
        double linear_vel;
        double angular_vel;
        double x;
        double l;
        double theta;
        double angle_difference;

        for(int i=0; i< bogie_poses_.size(); i++){
            GlobalOrd bogie = bogie_poses_[temp_].position;  //position of the shortest trajectory
            GlobalOrd base_position = sim_->BSTATION_LOC;

            double base_to_bogie = sim_->distance(bogie,base_position); //Variable to later check if bogie is out of range

            if(base_to_bogie <= sim_-> AIRSPACE_SIZE/2){                 //If bogie is in range, chase it
                angle_difference = angleDifference(friendly_pose, bogie);
                x = (bogie.y - friendly_pose.position.y)*cos(friendly_pose.orientation) - (bogie.x-friendly_pose.position.x)*sin(friendly_pose.orientation);
                l = sim_->distance(friendly_pose.position,bogie);
                theta = 2*x/(l*l);
                if(angle_difference>=TURNING_THRESHOLD)         //turn the friendly to match the destination path
                    return {sim_->V_TERM, MAX_OMEGA};

                else if(angle_difference<-TURNING_THRESHOLD) 
                    return{sim_->V_TERM,-MAX_OMEGA};

                else{                               //chase the bogie as max speed
                    linear_vel = sim_->MAX_V;
                    angular_vel = linear_vel*theta;
                    return {linear_vel,angular_vel};
                }
            }

            else{       
                //If the selected bogie is out of range, chase the base and check for another bogie
                angle_difference = angleDifference(friendly_pose,base_position);
                l = base_to_bogie;
                x = (base_position.y-friendly_pose.position.y)*cos(friendly_pose.orientation) - (base_position.x-friendly_pose.position.x)*sin(friendly_pose.orientation);
                theta = 2*x/(l*l);
                if(angle_difference>TURNING_THRESHOLD) 
                    return {sim_->V_TERM,MAX_OMEGA};

                else if(angle_difference<-TURNING_THRESHOLD) 
                    return{sim_->V_TERM,-MAX_OMEGA};

                else{
                    double linear_vel = sim_->MAX_V;
                    double angular_vel = linear_vel*theta;
                    return {linear_vel,angular_vel};
                }
            }
        }
}


vector<Pose> Synchronization::Bogie_Estimation(){
        bogie_poses_.clear();    //clear the previous position to make sure it won't display on the global map
        vector<GlobalOrd> bogie_positionitions;

        Pose friendly_coordinate = sim_->getFriendlyPose();                                     //friendly coordinate
        vector<RangeVelocityStamped> BaseBogie_RangeVelocity = BogieBaseRangeConsumer();        //range from bogie to base & velocity of bogie
        vector<RangeBearingStamped> BogieFriendly_RangeBearing = BogieFriendlyRangeConsumer();  //range from bogie to friendly and bearing of bogie
        Pose bogie;

        for(int i=0; i < BogieFriendly_RangeBearing.size(); i++ ){
            double alpha  = BogieFriendly_RangeBearing[i].bearing + friendly_coordinate.orientation;
            //calculate x-y position of all bogies
            GlobalOrd bogie_position;
            bogie_position.x = friendly_coordinate.position.x + BogieFriendly_RangeBearing[i].range*cos(alpha);  
            bogie_position.y = friendly_coordinate.position.y + BogieFriendly_RangeBearing[i].range*sin(alpha);

            bogie_positionitions.push_back(bogie_position);
        }
        
        if(point_container_.size()<CONTAINER_SIZE) 
                point_container_.push(bogie_positionitions); //if there is no data in the container, push the newest data
        
        else{   //swap the oldest data with the newest data, pop the old data and push the new one in to the container
                // point_container_.front() = point_container_.back();
                point_container_.pop();
                point_container_.push(bogie_positionitions);
            }

        vector<double> time_holder;
        for(int i = 0; i< BogieFriendly_RangeBearing.size(); i++){
                time_holder.push_back(BogieFriendly_RangeBearing[i].timestamp);
        }   
        if(timestamp_container_.size()<CONTAINER_SIZE) 
                timestamp_container_.push(time_holder);
        else{
                // timestamp_container_.front() = timestamp_container_.back();
                timestamp_container_.pop();
                timestamp_container_.push(time_holder);
        }
            
        bogie_poses_ = Estimate_FutureBogie_Poses(point_container_,timestamp_container_,TIME);
    
        range_container_.clear();
        for(int i=0; i < BogieFriendly_RangeBearing.size(); i++){
                range_container_.push_back(BogieFriendly_RangeBearing[i].range); 
        }

        double shortest_time;  
        vector<double> impact_times;
        double impact_time;
        double temp;
        //assume that the range from the bogie to friendly is a straight line
        for(int i=0; i<range_container_.size(); i++){
            //predicted that both bogie and friendly is head to head and run at 2 different speed
            //we can estimate the time they will meet each other
            impact_time = (range_container_[i]/(sim_->MAX_V + BaseBogie_RangeVelocity[i].velocity)); 
         
            impact_times.push_back(impact_time);
        }
        //find the location of the bogie that takes the shortest time to fly to
        shortest_time = impact_times[0];
        for (int i = 0; i < impact_times.size() ; i++)
        {
            if(shortest_time>impact_times[i]){
                shortest_time = impact_times[i];
                temp = i;
            }
        }
        temp_=temp;    //return the position of the destinated bogie
 
    return bogie_poses_;
}

vector<Pose> Synchronization::Estimate_FutureBogie_Poses(queue<vector<GlobalOrd>> point_container, queue<vector<double>> timestamp_container, double step_time){  
        vector<Pose> bogie_positions;                               //container to store all the calculated bogie pose

        vector<GlobalOrd> front_pose = point_container.front();      //get the first pose from the container
        vector<GlobalOrd> back_pose = point_container.back();      //get the second pose from the container

        vector<double> front_time = timestamp_container.front();     //get 1st the time stamp
        vector<double> back_time = timestamp_container.back();     //get 2nd the time stamp

        for(int i=0; i < front_pose.size(); i++){
            Pose bogie_position;
            double estimation_time = back_time[i] + step_time;

            /*function to calculate the estimate pose for all bogies using extrapolation formular*/

            bogie_position.position.x = front_pose[i].x + (estimation_time - front_time[i])/(back_time[i] - front_time[i])*(back_pose[i].x-front_pose[i].x);

            bogie_position.position.y = front_pose[i].y + (estimation_time - front_time[i])/(back_time[i] - front_time[i])*(back_pose[i].y-front_pose[i].y);

            bogie_position.orientation = atan2((back_pose[i].y-front_pose[i].y) , (back_pose[i].x-front_pose[i].x ));

            bogie_positions.push_back(bogie_position);
        }
    return bogie_positions;
}

void Synchronization::delay(int millisecond){    //delay time
        std::this_thread::sleep_for(std::chrono::milliseconds(millisecond));
}