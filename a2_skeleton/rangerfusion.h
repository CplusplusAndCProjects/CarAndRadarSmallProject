#ifndef RANGERFUSION_H
#define RANGERFUSION_H

#include <vector>
#include "rangerfusioninterface.h"
#include "rangerinterface.h"
#include  <tgmath.h>
#include "cell.h"
#include "ranger.h"
#include <math.h>
#include <iterator>
#include <bits/stdc++.h>

const double PI = 2*acos(0);
const int FOV = 20;
const int range = 180;
const int laser_FOV = 180;
const int sonar_FOV =20;
using namespace std;
class RangerFusion: public RangerFusionInterface
{
public:
  /**
    The Default constructor sets the cell centre to values within the #MAP_SIZE\n
    @sa RangerFusionInterface and @sa RangerInterface for more information
   */
  RangerFusion(std::vector<RangerInterface*> rangers);

  //!Inner working:
  //!Resize the raw_data_ 2D vector with the number of rangers created. Then populate each sub-vector with generateData method
  /**
   * @brief     Getter for pre-fused data
   * @return    2D vector of double of reading values
   */ 
  std::vector<vector<double>> getRawRangeData();

  /**
   * Member function sets Cells
   * @brief  getter for pre-fused data
   * @return The desired vector to store all cells
   */ 
  void setCells(std::vector<Cell*> cells);

  /**
   * Member function get the cell Cells
   * @brief  getter for pre-fused data
   * @return The desired vector to store all cells
   */ 
  vector<Cell*> getCells();

  /**
   * Member function get the rangers implimented
   * @brief  getter for pre-fused data
   * @return The desired vector to store all the rangers
   */ 
  std::vector<RangerInterface*> getRangers(); 

  /**
   * Member function set the cellside
   * @brief  setter for pre-fused data
   * @return The desired value of the cell side
   */ 
  double setCellside(double cellside);

  /**
   * Member function get the area covered by 2 sonars
   * @brief  getter for cone-type ranger total area
   * @return The desired value of the area covered by 2 sonars [m2]
   */ 
  double getScanningArea();

  /**
   * Member function get the Angular Reslusion of the laser
   * @brief  getter for the angular resolution
   * @return The value of the laser Angular resolution [m]
   */ 
  double get_AngularResolution();

  /**
   * Member function get the Angular Offset of the Sonar
   * @brief  getter for the angular offset
   * @return The deside vector to store all sonar's Angular Offset
   */ 
  vector<double> getAngualar_Offset();

  /**
   * Member function get the laser data
   * @brief  getter for the pre-fused laser data
   * @return The deside vector to store the laser data after the program implimented
   */ 
  vector<double> getLaserData();

  /**
   * Member function get the X coordinate of each laser line
   * @return The deside vector to store the laser X coordinate data after the program implimented
   */ 
  vector<double> getLaser_XCoordinate();

  /**
   * Member function get the Y coordinate of each laser line
   * @return The deside vector to store the laser Y coordinate data after the program implimented
   */
  vector<double> getLaser_YCoordinate();

  /**
   * Member function get the X coordinate of 2 oposite point of the cell
   * @return The deside vector to store X coordinate  of 2 oposite point of the cell
   */
  vector<double> getCellside_Xcoordinate(double x, double cellside);

  /**
   * Member function get the Y coordinate of 2 oposite point of the cell
   * @return The deside vector to store Y coordinate  of 2 oposite point of the cell
   */
  vector<double> getCellside_Ycoordinate(double y, double cellside);

  /**
   * Member function consider relative position of the laser and cell
   * @return The deside vector to store all of the states of cells
   */
  vector<cell::State> laser_fusion(double cell_centerX, double cell_centerY, double cellside, vector<double>cellside_Xcoordinate, vector<double>cellside_Ycoordinate, vector<double>Ranger, vector<double> Ranger_Xcoordinate, vector<double> Ranger_Ycoordinate);

  /**
   * Member function get the sonar data
   * @brief  Laser an Cell fusion
   * @return The deside vector to store the sonar data after the program implimented
   */ 
  vector<double> getSonarData();

  /**
   * Member function get the X coordinates of all sonars
   * @return The deside vector to store the sonar X coordinate data after the program implimented
   */ 
  vector<double> getSonar_Xcoordinate();

  /**
   * Member function get the Y coordinates of all sonars
   * @return The deside vector to store the sonar Y coordinate data after the program implimented
   */ 
  vector<double> getSonar_Ycoordinate();

  /**
   * Member function consider relative position between the sonar and cells
   * @brief  Sonar an Cell fusion
   * @return The deside vector to store all of the states of cells
   */
  vector<cell::State> SonarFusion(double cell_centerX, double cell_centerY, double cellside );

  /**
   * Member function Grab and Fuse data
   * @brief  Sonar, Laser and Cell Fusion
   * @return the state of all the Cell after data fusion [State]
   */
  void grabAndFuseData();

  /**
   * Member function Grab and Fuse data
   * @return the raw data before fusion [m]
   */
  vector<vector<double>> print_Data();
private:
  //
  vector<vector<double>> data_;   //!< This is to cater for getRawRangeData (which returns the raw data that was used for fusion))
  vector<RangerInterface*> rangers_; //!< A private copy of rangers @sa RangerInterface
  vector<Cell*> cells_;             //!< A private copy of cells @sa Cell
  vector<vector<double> > raw_data_; //!<A private copy of raw range data
  double area_;                     //!<A private copy of the raw area covered by 2 cones
  double cell_side_;                //!<A private copy of cellside  
  int resolution_;                  //!<A private copy of laser angular resolution 
  double offset1_;                  //!<A private copy of sonar1 Angular offset 
  double offset2_;                  //!<A private copy of sonar2 Angular offset
};

#endif // RANGERFUSION_H

