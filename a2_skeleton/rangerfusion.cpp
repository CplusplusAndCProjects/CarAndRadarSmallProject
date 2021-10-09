#include "rangerfusion.h"

//!<Ranger fusion contructor to hold all the rangers
RangerFusion::RangerFusion(std::vector<RangerInterface*> rangers)
{
    rangers_ = rangers;
}

//a vector to hold all the value of initialized rangers
std::vector<RangerInterface*> RangerFusion::getRangers()
{
    return rangers_;
}

//!<A vector to populate raw data
vector<vector<double> > RangerFusion::getRawRangeData()
{
    raw_data_.resize(rangers_.size());
    for(int i = 0;i<raw_data_.size();i++)
    {
        raw_data_[i]=(*rangers_[i]).generateData();
    }
    return raw_data_;
}


//!<Set up the cell
void RangerFusion::setCells(std::vector<Cell*> cells)
{
    cells_ = cells;
}

//!<A function to get all the cell implimented
vector<Cell*> RangerFusion::getCells()
{
    return cells_;
}

//<! A function to get the Angular offset of 2 sonars
vector<double> RangerFusion::getAngualar_Offset()
{
    vector<double> offset_;
    double offset;
    for(auto i : getRangers())
    {
        offset = i -> getAngularOffset();
        if( i->getFieldOfView() == sonar_FOV )
            offset_.push_back(offset);
    }
    
    for(int i = 0; i <  offset_.size(); i++)
    {
        offset1_ =  offset_[0];
        offset2_ =  offset_[1];
    }
    
    return offset_;
}

//<! A function to get the Angular resolution of the laser
double RangerFusion::get_AngularResolution()
{
    double resolution;
    for(auto i : getRangers())
    {
        if( i -> getFieldOfView() == laser_FOV)
        resolution_ = i -> getAngularResolution();
    }

    return resolution_;
}

//<! A function that return the area covered by 2 cones
double RangerFusion::getScanningArea()
{     
    double offset;
    double S;
    vector<double> sonar_data_ = getSonarData(); 
    double total_area = 0;
    getSonarData();
    getAngualar_Offset();

    for (auto i : sonar_data_)
    {
        S += (i*i*PI*FOV)/range;
            
    }

    if (abs(offset1_-offset2_) >= 20) //2 sonar lies seperately
    {
        total_area = S;
    }

    else        //if 2 sonar is duplicate to each other
    {
        double smallest_number = *min_element(sonar_data_.begin(), sonar_data_.end()); //find the smallest cone side
        double middle_angle = abs(abs(offset1_-offset2_)-FOV);                            //the angle between 2 dupplicate cone
        total_area = S - (smallest_number*smallest_number*middle_angle*PI )/range;
    }

    area_=total_area;
    return area_;
}

//<!A vector function return the sonar data
vector<double> RangerFusion::getSonarData()
{
    vector<double> sonar_data_;
     for(int i = 0; i < raw_data_.size();i++)
    {
        if(raw_data_[i].size()<2)
        {
            for(int j=0; j < raw_data_[i].size(); j++)
                sonar_data_.push_back(raw_data_[i][j]);
        }
    }
    return sonar_data_;
}

//<!A vector function return the laser data
std::vector<double> RangerFusion::getLaserData()
{
    vector<double> laser_data_;
    for(int i = 0; i < raw_data_.size();i++)
    {
        if(raw_data_[i].size()>2)
        {
            for(int j=0; j < raw_data_[i].size(); j++)
                laser_data_.push_back(raw_data_[i][j]);
        }
    }
    return laser_data_;
}

//<!A funtion to get the cellside inputed by the user
double RangerFusion::setCellside(double sellside)
{
    for(auto i = cells_.begin();i != cells_.end();i++)
    {
        (*i)->setSide(sellside);
        cell_side_= sellside;
    }
}

//<!A vector function to calculate the laser X coordinate
vector<double> RangerFusion::getLaser_XCoordinate()
{
    double offset;
    for(auto i : getRangers())
    {
        if(i->getFieldOfView()== laser_FOV)
        offset = i -> getAngularOffset();
    }
    double resolution = get_AngularResolution();
    vector<double> laser_data_ = getLaserData();
    vector<double> laser_Xcoordinate_;
    double x;
    for(int i = 0; i < getLaserData().size(); i++)
    {   
    
        if(i*resolution +offset == 90)
        {
            laser_Xcoordinate_.push_back(0);
        }
        else if( i*resolution + offset == 0  )
        {
            laser_Xcoordinate_.push_back(laser_data_[i]);
        }
        else if(i*resolution + offset == 180)
        {
            laser_Xcoordinate_.push_back(-laser_data_[i]);
        }
        else 
        {
            x = (laser_data_[i])*cos((i*resolution+offset)*PI/180);
            laser_Xcoordinate_.push_back(x);
        }
    }    
    return laser_Xcoordinate_;
}

//<!A vector function to calculate the laser Y coordinate
vector<double> RangerFusion::getLaser_YCoordinate()
{
    double offset;
    for(auto i : getRangers())
    {
        if(i->getFieldOfView()==laser_FOV)
        offset = i -> getAngularOffset();
    }
    double resolution=get_AngularResolution();
    vector<double> laser_Ycoordinate_;
    vector<double> laser_data_ = getLaserData();
    double y=0;
    for(int i = 0; i < getLaserData().size(); i++)
    {
        y = laser_data_[i]*sin((i*resolution+offset)*PI/180); //convert to radians
        laser_Ycoordinate_.push_back(y);
    }    
    return laser_Ycoordinate_;
}

//<! A function to calculate the 2 X coordinate of the cell side
vector<double> RangerFusion::getCellside_Xcoordinate(double x, double cellside)
{
    vector<double> cellside_Xcoordinate;
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
    cellside_Xcoordinate = {x1, x2};
    return cellside_Xcoordinate;
}

//<! A function to calculate the 2 Y coordinate of the cell side
vector<double> RangerFusion::getCellside_Ycoordinate(double y, double cellside)
{
    vector<double> cellside_Ycoordinate;
    double  y1, y2;

    y1 = y - 0.5*cellside;
    y2 = y + 0.5*cellside;
    cellside_Ycoordinate = {y1, y2};

    return cellside_Ycoordinate;
}

//<! A funtion that return the state between the laser and the cell
vector<cell::State> RangerFusion::laser_fusion(double cell_centerX, double cell_centerY, double cellside, vector<double>cellside_Xcoordinate, vector<double>cellside_Ycoordinate, vector<double>Ranger, vector<double> Ranger_Xcoordinate, vector<double> Ranger_Ycoordinate)
{
    getCellside_Xcoordinate(cell_centerX, cellside);
    getCellside_Ycoordinate(cell_centerY, cellside);
    vector<cell::State> final_state;
    int temp=0; //a temporary value to hold the temp value of the cell
    for(int i = 0; i < Ranger.size(); i++)
    {  
        double lineslope = Ranger_Ycoordinate[i]/Ranger_Xcoordinate[i];         //slope of the laser line
        double cellslope1 = cellside_Ycoordinate[0] / (cellside_Xcoordinate[0] + cellside); //slope of the line between center and one point on the cell
        double cellslope2 = cellside_Ycoordinate[1] / (cellside_Xcoordinate[1] - cellside);   //slope of the line between center and opposite point on the cell

        if (Ranger.size() == getLaserData().size() && cell_centerY-cellside/2<Ranger_Xcoordinate[Ranger.size()-1] ) //make sure that all the cell lies above the y axis when considering the laser fusion
        {
            temp = 0;
        }
        else
        {
            //checking the condition between each laser line and cells
            // if ((Ranger_Xcoordinate[i]>0 || Ranger_Xcoordinate[i]<0) && cell_centerY>-cellside/2 && Ranger_Ycoordinate[i] > 0)
            if ((Ranger_Xcoordinate[i]>0 || Ranger_Xcoordinate[i]<0)  && Ranger_Ycoordinate[i] > 0)
            { 
                if(((Ranger_Xcoordinate[i] > 0 && Ranger_Ycoordinate[i] > 0 && Ranger_Xcoordinate[i] > cellside_Xcoordinate[0] && Ranger_Ycoordinate[i] > cellside_Ycoordinate[0] && lineslope > cellslope1 && lineslope < cellslope2)
                    || (Ranger_Xcoordinate[i] < 0 && Ranger_Ycoordinate[i] > 0 && Ranger_Xcoordinate[i] < cellside_Xcoordinate[0] && Ranger_Ycoordinate[i] > cellside_Ycoordinate[0] && lineslope < cellslope1 && lineslope > cellslope2))
                    &&temp==0)
                {
                    temp = 1;
                }   
                if (((Ranger_Xcoordinate[i]>0 && Ranger_Ycoordinate[i]> 0 && Ranger_Xcoordinate[i] > cellside_Xcoordinate[0] && Ranger_Xcoordinate[i] < cellside_Xcoordinate[1] && Ranger_Ycoordinate[i] > cellside_Ycoordinate[0] && Ranger_Ycoordinate[i] < cellside_Ycoordinate[1]) 
                    || (Ranger_Xcoordinate[i]<0 &&Ranger_Ycoordinate[i] > 0 && Ranger_Xcoordinate[i] < cellside_Xcoordinate[0] && Ranger_Xcoordinate[i] > cellside_Xcoordinate[1] && Ranger_Ycoordinate[i] > cellside_Ycoordinate[0] && Ranger_Ycoordinate[i] < cellside_Ycoordinate[1] ))
                    &&temp==1)
                {
                    temp = 2;
                } 
                

            }

            // if((Ranger_Xcoordinate[i]==0) && cell_centerY > -cellside/2 && Ranger_Ycoordinate[i] > 0)
            if((Ranger_Xcoordinate[i]==0) && Ranger_Ycoordinate[i] > 0)
            { 
                
                if(Ranger_Ycoordinate[i] >= cellside_Ycoordinate[1] || Ranger_Ycoordinate[i] >= cellside_Ycoordinate[0]
                    &&temp == 0)
                {
                    temp = 1;
                }

                if ( (Ranger_Ycoordinate[i] <= cellside_Ycoordinate[1]) || ((Ranger_Ycoordinate[i] <= cellside_Ycoordinate[0]))
                    &&temp==1)
                {
                    temp = 2;
                }
            }

            if(Ranger_Ycoordinate[i]==0)
            {
                if(Ranger_Xcoordinate[i]>0)
                {   
                     
                    if(Ranger_Xcoordinate[i]>cellside_Xcoordinate[0] && temp ==0)
                        temp=1; 
                    if(Ranger_Xcoordinate[i]>cellside_Xcoordinate[0] && Ranger_Xcoordinate[i]<cellside_Xcoordinate[1]&&temp==1)
                        temp=2; 
                }

                if(Ranger_Xcoordinate[i]<0)
                {
                    
                    if(Ranger_Xcoordinate[i]<cellside_Xcoordinate[1])
                        temp=1;
                    if(Ranger_Xcoordinate[i]<cellside_Xcoordinate[0] && Ranger_Xcoordinate[i]>cellside_Xcoordinate[1] &&temp==1)
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

//<!A vector function to calculate the sonar X coordinate
vector<double>  RangerFusion::getSonar_Xcoordinate()
{
    vector<double> Sonar_Xcoordinate;
    vector<double> sonar_data_ = getSonarData();
    vector<double> offset = getAngualar_Offset();
    double x1, x2, x3, x4;

    for(int i = 0; i < sonar_data_.size(); i++)
    {
        x1 = sonar_data_[i]*cos((90+offset[i]-FOV/2)*PI/180);
        x2 = sonar_data_[i]*cos((90+offset[i]+FOV/2)*PI/180);
        Sonar_Xcoordinate.push_back(x1);
        Sonar_Xcoordinate.push_back(x2);
    }

    return Sonar_Xcoordinate;
}

//<!A vector function to calculate the sonar Y coordinate
vector<double>  RangerFusion::getSonar_Ycoordinate()
{
    vector<double> Sonar_Ycoordinate;
    vector<double> sonar_data_ = getSonarData();
    getAngualar_Offset();
    double y1, y2, y3, y4;

    for(int i = 0; i < sonar_data_.size(); i++)
    {
        y1 = sonar_data_[i]*sin((90+offset1_-FOV/2)*PI/180);
        y2 = sonar_data_[i]*sin((90+offset1_+FOV/2)*PI/180);
        Sonar_Ycoordinate.push_back(y1);
        Sonar_Ycoordinate.push_back(y2);
    }
    return Sonar_Ycoordinate;
}

//<! A funtion to check the relation between the sonar and the cell
vector<cell::State> RangerFusion::SonarFusion(double cell_centerX, double cell_centerY, double cellside)
{
    vector <cell::State> first_fuse;
    vector <cell::State> final_fuse;

    //consider 2 side of the cone is 2 laser line
    //first, I check the relation between the cell and 2 outer line of the cone
    first_fuse = laser_fusion(cell_centerX, cell_centerY, cellside,  getCellside_Xcoordinate(cell_centerX, cellside), getCellside_Ycoordinate(cell_centerY, cellside), 
                                getSonarData(), getSonar_Xcoordinate(),  getSonar_Ycoordinate());


    vector<cell::State> second_fuse;   

    vector<double> cell_Xcoordinate = getCellside_Xcoordinate(cell_centerX, cellside);
    vector<double> cell_Ycoordinate = getCellside_Ycoordinate(cell_centerY, cellside);
    vector<double> Sonar_Xcoordinate = getSonar_Xcoordinate();
    vector<double> Sonar_Ycoordinate = getSonar_Ycoordinate();
    vector<double> sonar_data = getSonarData();
  

    double cellslope1 = cell_Ycoordinate[0] / (cell_Xcoordinate[0] + cellside); //slope of the line between center and one point on the cell
    double cellslope2 = cell_Ycoordinate[1] / (cell_Xcoordinate[1]- cellside);   //slope of the line between center and opposite point on the cell
    int temp =0; 
    //calculate the slope of outer line of 2 sonars
    for(int i = 0; i < sonar_data.size(); i++)
    {
        double lineslope1 = Sonar_Ycoordinate[2*i] / Sonar_Xcoordinate[2*i];
        double lineslope2 = Sonar_Ycoordinate[2*i+1] / Sonar_Xcoordinate[2*i+1];


        double distance = sqrt(pow(cell_centerX,2) +pow(cell_centerY,2));   //calculate the distance between the cell and the center 0 
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
        second_fuse.push_back(cell::UNKNOWN);
    }
    if(temp==1)
    {
        second_fuse.push_back (cell::FREE);
    }

    if(temp==2)
    {
        second_fuse.push_back (cell::OCCUPIED);
    }

    //compare the state of 2 fuse
    for(int i = 0; i < first_fuse.size(); i++)
    {
        if((first_fuse[i]==0 && second_fuse[i]==0) || (first_fuse[i]==1 && second_fuse[i]==1) ||(first_fuse[i]==-1 && second_fuse[i]==-1))
        {
            final_fuse.push_back(first_fuse.at(i));
        }

        else if((first_fuse[i]==0 && second_fuse[i]==1) || (first_fuse[i]==1 && second_fuse[i]==0))
        {
            final_fuse.push_back(cell::FREE);
        }

        else if((first_fuse[i]==-1 && second_fuse[i]==0) || (first_fuse[i]==0 || second_fuse[i]==-1) || (first_fuse[i]==1 || second_fuse[i]==-1) ||(first_fuse[i]==-1 || second_fuse[i]==1))
        {
            final_fuse.push_back(cell::OCCUPIED);
        }
        else
            final_fuse.push_back(cell::UNKNOWN);
    }

    //return the final result 
    return final_fuse;
}

//<! The final function to set the state for the cell with all the rangers
void RangerFusion::grabAndFuseData()
{
    double x, y;
    data_ = getRawRangeData();
    getRangers();
    get_AngularResolution();
    setCells(cells_);
    setCellside(cell_side_);

    for (auto it = cells_.begin();it != cells_.end(); it++ ) //Return the state for each cell
    {   
            (*it)->getCentre(x,y);
            (*it)->setSide(cell_side_);

            vector<cell::State> sonarfusion= SonarFusion(x, y, cell_side_);
            vector<cell::State> laserfusion= laser_fusion(x,y,(*it)->getSide(), getCellside_Xcoordinate( x, (*it)->getSide()),
                 getCellside_Xcoordinate( y, (*it)->getSide()), getLaserData(), getLaser_XCoordinate(),  getLaser_YCoordinate());

            vector<cell::State> final_fuse;
            for(int i = 0; i < sonarfusion.size(); i ++)
            {
                if((sonarfusion[i]==0 && laserfusion[i] == 0) || (sonarfusion[i]==0 && laserfusion[i] == 1) || (sonarfusion[i]==-1 && laserfusion[i] == -1))
                {
                    final_fuse.push_back(sonarfusion[i]);
                }
                if((sonarfusion[i]== 0 && laserfusion[i]==1 ) || (sonarfusion[i]== 1 && laserfusion[i]== 0))
                {
                    final_fuse.push_back(cell::FREE);
                }
                if((sonarfusion[i]== 0 && laserfusion[i]== -1 ) || (sonarfusion[i]== -1 && laserfusion[i]== 0)||
                     (sonarfusion[i]== -1 && laserfusion[i]==1 ) || (sonarfusion[i]== 1 && laserfusion[i]== -1))
                {
                    final_fuse.push_back(cell::OCCUPIED);
                }
                
            }
            for( auto i : final_fuse)
            {
                (*it) -> setState(i);
            }
    }
}

vector<vector<double>> RangerFusion::print_Data()
{
    return raw_data_;
}