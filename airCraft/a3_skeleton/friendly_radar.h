#ifndef FRIENDLY_RADAR
#define FRIENDLY_RADAR
#include "radar_interface.h"


/*!
 * \brief The friendly_Radar class, inherit from radar
 * @author Trong Duy Bui
 * @date 2nd October 2020
*/

class Friendly_Radar: public Radar_Interface
{
public:
    /**
     * @brief Constructor for the friendly radar, takes in a shared pointer
     * @param sim: Shared pointer to simulator object
     */
    Friendly_Radar(shared_ptr<Simulator> & sim);
    /**
     * @brief Return range, bearing and time data with a rate of 10Hz
     * @return Data of RangeVelocityStamped type
     */
    vector<RangeBearingStamped> getRangeBearingData();

     /**
     * @brief Return range, velocity and bearing data with a rate of 10Hz
     * @return Data of RangeVelocityStamped type
     */
    vector<RangeVelocityStamped> getRangeVelocityData();
    /**
     * @brief Set simulator for the passed object
     * @param sim
     */
    void setSimulator(shared_ptr<Simulator> & sim);


private:
    vector<RangeBearingStamped> friendly_data_;                     //!< Vector holder for generated data
    shared_ptr<Simulator> sim_;                            //!< Shared pointer

};

#endif // FRIENDLY_RADAR

