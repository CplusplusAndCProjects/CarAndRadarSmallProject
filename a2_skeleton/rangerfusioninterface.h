#ifndef RANGERFUSIONINTERFACE_H
#define RANGERFUSIONINTERFACE_H

#include <vector>
#include "rangerinterface.h"
#include "cell.h"

/**
 * @brief Specifies the required interface for your RangerFusion class your ranger fusion
// class must inherit from it. <b> You MUST NOT edit this file </b>.
 * 
 */
class RangerFusionInterface
{
public:
    RangerFusionInterface();

    /**
     * @brief Accepts the container of cells.
     *
     * @param cells
     */
    virtual void setCells(std::vector<Cell*> cells) = 0;

    /**
     * @brief Calls each ranger to generate data and combines range data with provided container of cells. Generates a 'fusion' of the data based on collision conditions as descibed in Assignment 2 specification
     *
     */
    virtual void grabAndFuseData() = 0;

    /**
     * @brief Returns the raw data from any sensors in the ranger container within a vector of vectors
     * The raw data is updated every time a new fusion is requested. The raw data will match the preceeding fusion if it is called between fusions. If no fusion has occured the vector shall be empty.
     *
     * @return std::vector<std::vector<double>>  the outer elements of the vector related to the rangers, the inner elements of vector the respective range readings

     *
     * @sa grabAndFuseData
     */
    virtual std::vector<std::vector<double>> getRawRangeData() = 0;

    /**
     * @brief Returns the total scanning area possible with CONE based scanners supplied
     * A union of all areas https://en.wikipedia.org/wiki/Union_(set_theory)
     *
     * @return double Total area coverage
     *
     * @sa grabAndFuseData
     */
    virtual double getScanningArea() = 0;


};

#endif // RANGERFUSIONINTERFACE_H
