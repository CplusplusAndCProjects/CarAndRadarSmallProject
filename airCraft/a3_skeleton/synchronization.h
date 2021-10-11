#ifndef SYNCHRONIZATION
#define SYNCHRONIZATION
#include "radar_interface.h"
#include "base_radar.h"
#include "friendly_radar.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <vector>

/*!
 * \brief   Get the data from 2 radars and calculating for fusion purposes
 * \author  Trong Duy Bui
 * \date    2nd October, 2020
*/

struct Speed{
    double linear_velocity;
    double angular_velocity;
};

class Synchronization{
public:
    const double PI = 3.14;                                      //!< Pi constant
    const double G = 9.8;                                        //!< G constant
    const double MAX_OMEGA = ((sim_->MAX_G*G)/sim_->V_TERM);     //!< Maximum angular velocity
    const double TIME = 100;                                     //!< Time step for extrapolation (millisecond)
    const int CONTAINER_SIZE = 10;                               //!< Number of elements for extrapolation
    const int WATCHDOG_TIMER = 50;                               //!< The amount of delay required by the watchdog timer
    const int TIME_BETWEEN_POSE = 100;                           //!< Delay for calculator thread
    const double TURNING_THRESHOLD = PI/95;                      //!< Turning threshold of the friendly aircraft

    /**
     * @brief Constructor to store all radars data to initialise
     * @param radars: container to store all radars
     * @param sim: shared pointer of simulator
    */
    Synchronization(vector<Radar_Interface*> radars, shared_ptr<Simulator> & sim);

    /**
     * @brief Constructor to store base and friendly radar
     * @param radars
    */
    void RadarContainer(vector<Radar_Interface*> radars);

     /**
     * @brief get the range bogie from base and Velocity of bogies
    */
    void BogieBaseRangeProducer();

    /**
     * @brief Get the range from friendly to bogie and bearing data of friendly
    */
    void BogieFriendlyRangeProducer();

    /**
     * @brief Constructor to get the latest value from base radar
     * @return Value of RangeBearingStamped (range, bearing, timestamp) type
    */
    vector<RangeVelocityStamped> BogieBaseRangeConsumer();

    /**
     * @brief Constructor to get the latest data from the friendly radar
     * @return Value of RangeBearingStamped (range, bearing, timestamp) type
    */
    vector<RangeBearingStamped> BogieFriendlyRangeConsumer();
    
    /**
     * @brief Calculate the angular difference between a pose and a point in airspace
     * @param pose: Pose of an object
     * @param point: Position of the destination
     * @return Angle difference between 2 given parameters
    */
    double angleDifference(Pose pose, GlobalOrd o1);
    
    /**
     * @brief Constructor to drive the friendly to the desire bogie
     * @return linear and angular velocity of friendly
    */
    Speed Pure_Pursuit(const std::shared_ptr<Simulator> & sim);

    /**
     * @brief Estimate the future pose of all bogie to make the estimation slightly ahead
     * @param point_container vector to store all the pose for estimation purpose
     * @param timestamp_container vector to store all the timestamp for estimation purpose
     * @param step_time The time for the future estimation  
     * @return
    */
    vector<Pose> Estimate_FutureBogie_Poses(queue<vector<GlobalOrd>> point_container, queue<vector<double>> timestamp_container, double step_time);


    /**
     * @brief Constructor to find all the bogie in space and estimate its Pose
     * @return a vector contain all bogie pose and the most efficient one to chase
    */
    vector<Pose> Bogie_Estimation();

    /**
     * @brief Timer
    */
    void delay(int millisecond);

    int trajectory_estimator();



private:
    vector<Radar_Interface*> radar_container_;              //!< Container to store all the initialised radar
    int base_pos_;                                          //!< Position of the base radar in the initialised container
    int friendly_pos_;                                      //!< Position if the friendly radar in the initialised container
    mutex baseradar_mutex_;                                 //!< Mutex variable to protect the base radar data
    mutex friendlyradar_mutex_;                             //!< Mutex variable to protect the friendly radar
    mutex sync_mutex_;                                      //!< Mutex variable for syncronization
    queue<vector<RangeVelocityStamped>> base_buffer_;       //!< Variable to check the existance of the base radar datar
    queue<vector<RangeBearingStamped>> friendly_buffer_;    //!< Variable to check the existance of the friendly radar datar
    queue<vector<double>> timestamp_container_;             //!< Container to store all timestamp
    condition_variable base_cv_;                            //!< Condition variable to communicate between base radar and friendly radar
    condition_variable friendly_cv_;                        //!< Condition variable to communicate between friendly radar and base radar
    bool empty_ = true;                                     //!< Variable flag to check the existance of data
    shared_ptr<Simulator> sim_;                             //!< Share poiner of all variable
    vector<Pose> bogie_poses_;                              //!< Container for all calculated bogie pose
    int temp_=0;                                            //!< Position of the chosen bogie to chase
    queue<vector<GlobalOrd>> point_container_;              //!< Container of all pre-future estimation pose
    vector<double> range_container_;                        //!< Container of all friendly-bogie range
    vector<double> velocity_container_;
};

#endif // SYNCHRONIZATION