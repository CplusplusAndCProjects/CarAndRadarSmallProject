#ifndef BASE_RADAR
#include "radar_interface.h"
#include <iostream>
#define BASE_RADAR

/*!
 * \brief The base radar class, inherit from radar interface
 * @author Trong Duy Bui 
 * @date 2nd October 2020
*/

class Base_Radar: public Radar_Interface
{
public:
    /**
     * @brief Constructor with passed in pointer to simulator object
     * @param sim: Shared pointer to simulator object
     */
     
    Base_Radar(shared_ptr<Simulator> & sim);

    /**
     * @brief Return the range, velocity and timestamped data every 100HZ
     * @return  data RangeVelocityStamped type
     */
    vector<RangeVelocityStamped> getRangeVelocityData();

    /**
     * @brief Return the range, bearing adn timestamped data at 100hz rate
     * @return
     */
    vector<RangeBearingStamped> getRangeBearingData();
    
    /**
     * @brief Simulator setter
     * @param sim
     */
    void setSimulator(shared_ptr<Simulator> & sim);


private:
    vector<RangeVelocityStamped> base_data_;                     //!< Vector for generated data
    shared_ptr<Simulator> sim_;                            //!< Shared pointer

};

#endif // BASE_RADAR

