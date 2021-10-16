#ifndef RADAR
#include "simulator.h"
#define RADAR

using namespace std;

/*!
 * \brief The base class for radars
 */

class Radar_Interface
{
public:
    /**
     * @brief Constructor for the base class of all radars
     */
    Radar_Interface();
    /**
     * @brief Abstract method for data getting
     * @return Data of RangeVelocityStamped type
     */
    virtual vector<RangeVelocityStamped> getRangeVelocityData() = 0;

    /**
     * @brief Abstract method for data getting
     * @return Data of RangeBearingStamped type
     */
    virtual vector<RangeBearingStamped> getRangeBearingData() = 0;

    /**
     * @brief Setter for passed in simulator
     * @param Shared pointer to simulator class
     */
    virtual void setSimulator(shared_ptr<Simulator> & sim) = 0;
    /**
     * @brief Return the model
     * @return A string
     */
    string getModel();
    
protected:
    string model_;              //!< Attribute to keep the model
};

#endif // RADAR

