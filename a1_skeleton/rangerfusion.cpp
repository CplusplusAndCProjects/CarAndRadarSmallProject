#include"rangerfusion.h"

//!<Ranger fusion contructor to hold all the rangers
RangerFusion::RangerFusion(std::vector<RangerInterface*> rangers)
{
    mRangers = rangers;
}

//a vector to hold all the value of initialized rangers
std::vector<RangerInterface*> RangerFusion::getRangers()
{
    return mRangers;
}

//!<A vector to populate raw data
vector<vector<double> > RangerFusion::getRawRangeData()
{
    mRawData.resize(mRangers.size());
    for(int i = 0;i<mRawData.size();i++)
    {
        mRawData[i]=(*mRangers[i]).generateData();
    }
    return mRawData;
}


//!<Set up the cell
void RangerFusion::setCells(std::vector<Cell*> cells)
{
    mCells = cells;
}

//!<A function to get all the cell implimented
vector<Cell*> RangerFusion::getCells()
{
    return mCells;
}

//<! A function to get the Angular offset of 2 sonars
vector<double> RangerFusion::getAngualarOffset()
{
    vector<double> offsets;
    double offset;
    for(auto i : getRangers())
    {
        offset = i -> getSensorPose().theta;
        if( i->getFieldOfView() == FUSION_SONAR_FIELD_OF_VIEW )
            offsets.push_back(offset);
    }

    for(int i = 0; i <  offsets.size(); i++)
    {
        mSonarOffset1 =  offsets[0];
        msonarOffset2 =  offsets[1];
    }
    return offsets;
}
vector<ranger::SensorPose> RangerFusion::getSensorPoses(){
    vector<ranger::SensorPose> sensorPoses;
    ranger::SensorPose sensorPose;
    for(auto i : getRangers())
    {
        sensorPose = i -> getSensorPose();
        if( i->getFieldOfView() == FUSION_SONAR_FIELD_OF_VIEW )
            sensorPoses.push_back(sensorPose);
    }

    for(int i = 0; i <  sensorPoses.size(); i++)
    {
        mSonarSensorPose1 =  sensorPoses[0];
        mSonarSensorPose2 =  sensorPoses[1];
    }
    return sensorPoses;
}
//<! A function to get the Angular resolution of the laser
double RangerFusion::getAngularResolution()
{
    double resolution;
    for(auto i : getRangers())
    {
        if( i -> getFieldOfView() == FUSION_LASER_FIELD_OF_VIEW)
            mLaserResolution = i -> getAngularResolution();
    }

    return mLaserResolution;
}

//<! A function that return the area covered by 2 cones
double RangerFusion::getScanningArea()
{
    double offset;
    double S;
    vector<double> sonarData = getSonarData();
    double total_area = 0;
    getSonarData();
    vector<ranger::SensorPose> sensorPose = getSonarPose();
    for (auto i : sonarData)
    {
        S += (i*i*PI*FUSION_FIELD_OF_VIEW)/FUSION_RANGE;

    }

    if (abs(sensorPose[0].theta-sensorPose[1].theta) >= 20) //2 sonar lies seperately
    {
        total_area = S;
    }

    //if 2 sonar is duplicate to each other
    else
    {
        double smallest_number = *min_element(sonarData.begin(), sonarData.end()); //find the smallest cone side
        double middle_angle = abs(abs(sensorPose[0].theta-sensorPose[1].theta)-FUSION_FIELD_OF_VIEW);                            //the angle between 2 dupplicate cone
        total_area = S - (smallest_number*smallest_number*middle_angle*PI )/FUSION_RANGE;
    }

    mRawArea = total_area;
    return mRawArea;
}

//<!A vector function return the sonar data
vector<double> RangerFusion::getSonarData()
{
    vector<double> sonar_data;
     for(int i = 0; i < mRawData.size();i++)
    {
        if(mRawData[i].size()<2)
        {
            for(int j=0; j < mRawData[i].size(); j++)
                sonar_data.push_back(mRawData[i][j]);
        }
    }
    return sonar_data;
}

//<!A vector function return the laser data
std::vector<double> RangerFusion::getLaserData()
{
    vector<double> laserData;
    for(int i = 0; i < mRawData.size();i++)
    {
        if(mRawData[i].size()>2)
        {
            for(int j=0; j < mRawData[i].size(); j++)
                laserData.push_back(mRawData[i][j]);
        }
    }
    return laserData;
}

//<!A funtion to get the cellside inputed by the user
double RangerFusion::setCellside(double sellside)
{
    for(auto i = mCells.begin();i != mCells.end();i++)
    {
        (*i)->setSide(sellside);
        mCellSide= sellside;
    }
}

//<!A vector function to calculate the laser Pose

vector<ranger::SensorPose> RangerFusion::getLaserPose(){
    vector<ranger::SensorPose> laserPoses;
    ranger::SensorPose laserPose;

    for(auto i : getRangers())
    {
        if(i->getFieldOfView()== FUSION_LASER_FIELD_OF_VIEW)
            laserPose = i -> getSensorPose();

    }
    double resolution = getAngularResolution();

    vector<double> laserData = getLaserData();

    for(int i = 0; i < getLaserData().size(); i++)
    {

        if(i*resolution +laserPose.theta == 90)
        {
            laserPose.x = 0;
            //laserXcoordinate.push_back(0);
        }
        else if( i*resolution + laserPose.theta == 0  )
        {
            laserPose.x = laserData[i];
            //laserXcoordinate.push_back(laserData[i]);
        }
        else if(i*resolution + laserPose.theta == 180)
        {
            laserPose.x = -laserData[i];
            //laserXcoordinate.push_back(-laserData[i]);
        }
        else
        {
            laserPose.x = (laserData[i])*cos((i*resolution+laserPose.theta)*PI/180);
            //laserXcoordinate.push_back(x);
        }

        laserPose.y = laserData[i]*sin((i*resolution + laserPose.theta)*PI/180); //convert to radians

        laserPoses.push_back(laserPose);

    }
    return laserPoses;

}
//<! A function to calculate the 2 X coordinate of the cell side
vector<double> RangerFusion::getCellSideXcoordinate(double x, double cellside)
{
    vector<double> cellsideXcoordinate;
    double  x1, x2;
    if( x >= 0)
    {
        x1 = x - 0.5*cellside;
        x2 = x + 0.5*cellside;

    }

    if( x < 0)
    {
        x1 = x + 0.5*cellside;
        x2 = x - 0.5*cellside;

    }
    cellsideXcoordinate = {x1, x2};
    return cellsideXcoordinate;
}

//<! A function to calculate the 2 Y coordinate of the cell side
vector<double> RangerFusion::getCellsideYcoordinate(double y, double cellside)
{
    vector<double> cellsideYcoordinate;
    double  y1, y2;

    y1 = y - 0.5*cellside;
    y2 = y + 0.5*cellside;
    cellsideYcoordinate = {y1, y2};

    return cellsideYcoordinate;
}

//<! A funtion that return the state between the laser and the cell
vector<cell::State> RangerFusion::laserFusion(double cellCenterX, double cellCenterY, double cellside, vector<double>cellsideXcoordinate, vector<double>cellsideYcoordinate, vector<double>Ranger,
                                vector<ranger::SensorPose> sensorPoses)
{
    getCellSideXcoordinate(cellCenterX, cellside);
    getCellsideYcoordinate(cellCenterY, cellside);
    vector<cell::State> final_state;
    int temp=0; //a temporary value to hold the temp value of the cell
    for(int i = 0; i < Ranger.size(); i++)
    {
        double lineslope = sensorPoses[i].y/sensorPoses[i].x;         //slope of the laser line
        double cellslope1 = cellsideYcoordinate[0] / (cellsideXcoordinate[0] + cellside); //slope of the line between center and one point on the cell
        double cellslope2 = cellsideYcoordinate[1] / (cellsideXcoordinate[1] - cellside);   //slope of the line between center and opposite point on the cell

        if (Ranger.size() == getLaserData().size() && cellCenterY-cellside/2<sensorPoses[Ranger.size()-1].x ) //make sure that all the cell lies above the y axis when considering the laser fusion
        {
            temp = 0;
        }
        else
        {
            //checking the condition between each laser line and cells
            if ((sensorPoses[i].x>0 || sensorPoses[i].x<0)  && sensorPoses[i].y > 0)
            {
                if(((sensorPoses[i].x > 0 && sensorPoses[i].y > 0 && sensorPoses[i].x > cellsideXcoordinate[0] && sensorPoses[i].y > cellsideYcoordinate[0] && lineslope > cellslope1 && lineslope < cellslope2)
                    || (sensorPoses[i].x < 0 && sensorPoses[i].y > 0 && sensorPoses[i].x < cellsideXcoordinate[0] && sensorPoses[i].y > cellsideYcoordinate[0] && lineslope < cellslope1 && lineslope > cellslope2))
                    &&temp==0)
                {
                    temp = 1;
                }
                if (((sensorPoses[i].x>0 && sensorPoses[i].y> 0 && sensorPoses[i].x > cellsideXcoordinate[0] && sensorPoses[i].x < cellsideXcoordinate[1] && sensorPoses[i].y > cellsideYcoordinate[0] && sensorPoses[i].y < cellsideYcoordinate[1])
                    || (sensorPoses[i].x<0 &&sensorPoses[i].y > 0 && sensorPoses[i].x < cellsideXcoordinate[0] && sensorPoses[i].x > cellsideXcoordinate[1] && sensorPoses[i].y > cellsideYcoordinate[0] && sensorPoses[i].y < cellsideYcoordinate[1] ))
                    &&temp==1)
                {
                    temp = 2;
                }


            }

            if((sensorPoses[i].x==0) && sensorPoses[i].y > 0)
            {

                if(sensorPoses[i].y >= cellsideYcoordinate[1] || sensorPoses[i].y >= cellsideYcoordinate[0]
                    &&temp == 0)
                {
                    temp = 1;
                }

                if ( (sensorPoses[i].y <= cellsideYcoordinate[1]) || ((sensorPoses[i].y <= cellsideYcoordinate[0]))
                    &&temp==1)
                {
                    temp = 2;
                }
            }

            if(sensorPoses[i].y==0)
            {
                if(sensorPoses[i].x>0)
                {

                    if(sensorPoses[i].x>cellsideXcoordinate[0] && temp ==0)
                        temp=1;
                    if(sensorPoses[i].x>cellsideXcoordinate[0] && sensorPoses[i].x<cellsideXcoordinate[1]&&temp==1)
                        temp=2;
                }

                if(sensorPoses[i].x<0)
                {

                    if(sensorPoses[i].x<cellsideXcoordinate[1])
                        temp=1;
                    if(sensorPoses[i].x<cellsideXcoordinate[0] && sensorPoses[i].x>cellsideXcoordinate[1] &&temp==1)
                    {
                        temp=2;
                    }
                }
            }
        }

    }

        //return the final state of each cell
    if (temp == 0)
    {
        final_state.push_back(cell::UNKNOWN);
    }
    if (temp == 1)
    {
        final_state.push_back(cell::FREE);
    }
    if (temp == 2)
    {
        final_state.push_back(cell::OCCUPIED);
    }
    return final_state;
}

//<!A vector function to calculate the sonar Pose
vector<ranger::SensorPose> RangerFusion::getSonarPose(){
    vector<ranger::SensorPose> sonarPoses = getSensorPoses();
    vector<double> sonarData = getSonarData();
    //vector<double> offset = getAngualarOffset();
        double x1, x2, x3, x4, y1, y2, y3, y4;

        for(int i = 0; i < sonarData.size(); i++)
    {
        ranger::SensorPose sonarPose1,sonarPose2;
        sonarPose1.x = sonarData[i]*cos((90+sonarPoses[i].theta - FUSION_FIELD_OF_VIEW/2)*PI/180);
        sonarPose1.y = sonarData[i]*sin((90+sonarPoses[0].theta - FUSION_FIELD_OF_VIEW/2)*PI/180);

        sonarPose2.x = sonarData[i]*cos((90+sonarPoses[i].theta + FUSION_FIELD_OF_VIEW/2)*PI/180);
        sonarPose2.y = sonarData[i]*sin((90+sonarPoses[0].theta + FUSION_FIELD_OF_VIEW/2)*PI/180);
        sonarPoses.push_back(sonarPose1);
        sonarPoses.push_back(sonarPose2);
    }
    return sonarPoses;
}
//<! A funtion to check the relation between the sonar and the cell
vector<cell::State> RangerFusion::SonarFusion(double cellCenterX, double cellCenterY, double cellside)
{
    vector <cell::State> firstFuse;
    vector <cell::State> finalFuse;

    //consider 2 side of the cone is 2 laser line
    //first, I check the relation between the cell and 2 outer line of the cone
    firstFuse = laserFusion(cellCenterX, cellCenterY, cellside,  getCellSideXcoordinate(cellCenterX, cellside), getCellsideYcoordinate(cellCenterY, cellside),
                                getSonarData(), getSonarPose());
    vector<cell::State> secondFuse;

    vector<double> cell_Xcoordinate = getCellSideXcoordinate(cellCenterX, cellside);
    vector<double> cell_Ycoordinate = getCellsideYcoordinate(cellCenterY, cellside);
    vector<ranger::SensorPose> sensorPoses = getSonarPose();
    vector<double> sonar_data = getSonarData();

    double cellslope1 = cell_Ycoordinate[0] / (cell_Xcoordinate[0] + cellside); //slope of the line between center and one point on the cell
    double cellslope2 = cell_Ycoordinate[1] / (cell_Xcoordinate[1]- cellside);   //slope of the line between center and opposite point on the cell
    int temp =0;
    //calculate the slope of outer line of 2 sonars
    for(int i = 0; i < sonar_data.size(); i++)
    {
        double lineslope1 = sensorPoses[2*i].y / sensorPoses[2*i].x;
        double lineslope2 = sensorPoses[2*i+1].y / sensorPoses[2*i+1].x;


        double distance = sqrt(pow(cellCenterX,2) +pow(cellCenterY,2));   //calculate the distance between the cell and the center 0
        double cell_outerCircle = sqrt(2)*cellside/2;


        //check if the cell lies inside the cone
            if((lineslope1>=0 && lineslope2>=0 && cellslope1 > lineslope1 && cellslope2 < lineslope2) || (lineslope1<0 && lineslope2<0 && cellslope1 < lineslope1 && cellslope2 > lineslope2)
                || (lineslope1 > 0 && lineslope2 <0 && cellslope1 > lineslope1 && cellslope2 > lineslope2) || (lineslope1 < 0 && lineslope2 > 0 && cellslope1 < lineslope1 && cellslope2 < lineslope2)
                &&temp==0)
            {
                temp=1;     //if the line lies inside the cone, change the cell stage to free
            }

            //check if the cell is occupy with the cone
            if ((sonar_data[i]+ cell_outerCircle <= distance) && temp ==1)
            {
                temp = 2;   //if the outer circle of the cell intercept with the cone at 2 more point and it lies inside the cone, then, change the cell stage to occupy
            }

    }

    //set state for the 2nd fuse
    if(temp==0)
    {
        secondFuse.push_back(cell::UNKNOWN);
    }
    if(temp==1)
    {
        secondFuse.push_back (cell::FREE);
    }

    if(temp==2)
    {
        secondFuse.push_back (cell::OCCUPIED);
    }

    //compare the state of 2 fuse
    for(int i = 0; i < firstFuse.size(); i++)
    {
        if((firstFuse[i]==0 && secondFuse[i]==0) || (firstFuse[i]==1 && secondFuse[i]==1) ||(firstFuse[i]==-1 && secondFuse[i]==-1))
        {
            finalFuse.push_back(firstFuse.at(i));
        }

        else if((firstFuse[i]==0 && secondFuse[i]==1) || (firstFuse[i]==1 && secondFuse[i]==0))
        {
            finalFuse.push_back(cell::FREE);
        }

        else if((firstFuse[i]==-1 && secondFuse[i]==0) || (firstFuse[i]==0 || secondFuse[i]==-1) || (firstFuse[i]==1 || secondFuse[i]==-1) ||(firstFuse[i]==-1 || secondFuse[i]==1))
        {
            finalFuse.push_back(cell::OCCUPIED);
        }
        else
            finalFuse.push_back(cell::UNKNOWN);
    }

    //return the final result
    return finalFuse;
}

//<! The final function to set the state for the cell with all the rangers
void RangerFusion::grabAndFuseData()
{
    double x, y;
    mData = getRawRangeData();
    // cout<<"mrawData size: "<< (mData.size());
    // for(int i =0; i <mData.size(); i++){
    //     cout<<"mData size ["<<i<<"] ="<< mData[i].size();
    // }
    getRangers();
    getAngularResolution();
    setCells(mCells);
    setCellside(mCellSide);

    for (auto it = mCells.begin();it != mCells.end(); it++ ) //Return the state for each cell
    {
            (*it)->getCentre(x,y);
            (*it)->setSide(mCellSide);

            vector<cell::State> sonarfusion= SonarFusion(x, y, mCellSide);
            vector<cell::State> laserfusion= laserFusion(x,y,(*it)->getSide(), getCellSideXcoordinate( x, (*it)->getSide()),
                 getCellSideXcoordinate( y, (*it)->getSide()), getLaserData(), getSonarPose());

            vector<cell::State> finalFuse;
            for(int i = 0; i < sonarfusion.size(); i ++)
            {
                if((sonarfusion[i]==0 && laserfusion[i] == 0) || (sonarfusion[i]==0 && laserfusion[i] == 1) || (sonarfusion[i]==-1 && laserfusion[i] == -1))
                {
                    finalFuse.push_back(sonarfusion[i]);
                }
                if((sonarfusion[i]== 0 && laserfusion[i]==1 ) || (sonarfusion[i]== 1 && laserfusion[i]== 0))
                {
                    finalFuse.push_back(cell::FREE);
                }
                if((sonarfusion[i]== 0 && laserfusion[i]== -1 ) || (sonarfusion[i]== -1 && laserfusion[i]== 0)||
                     (sonarfusion[i]== -1 && laserfusion[i]==1 ) || (sonarfusion[i]== 1 && laserfusion[i]== -1))
                {
                    finalFuse.push_back(cell::OCCUPIED);
                }

            }
            for( auto i : finalFuse)
            {
                (*it) -> setState(i);
            }
    }
}

vector<vector<double>> RangerFusion::printData()
{
    return mRawData;
}
