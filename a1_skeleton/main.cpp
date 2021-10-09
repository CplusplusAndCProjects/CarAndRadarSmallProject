#include <iostream>
#include <limits>
#include "laser.h"
#include "ranger.h"
#include "sonar.h"
#include "rangerfusion.h"

#include <set>

using namespace std;

int main()
{
    system("clear");
    // FusionMethod method;
    Laser laser;
    Sonar sonar1;
    Sonar sonar2;
    int counter = 0;                        //Counter for the number of data generated so far
    int resolution = 10;   //Variable to hold the user input
    double  laserOffset, sonar1Offset, sonar2Offset, laserX, laserY, sonar1X, sonar1Y, sonar2X, sonar2Y;
    int number_of_cell;
    double cellside;
    double x, y;
    std::vector<Cell*> cells;
    Cell cell;
    cout<<"Please insert angular resolution for the laser, it should be either 10 or 30 degrees:"<<endl;
    cin>>resolution;
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');


    if(laser.setAngularResolution(resolution)) cout<<"Your laser is set to "<<resolution<<" degrees angular resolution."<<endl;
    else cout<<"Invalid input, default 10 degrees angular resolution will be used."<<endl;


    cout<<"Please insert laser pose for laser: "<<endl;
    cout<<"angular offset: "<<endl;
    cin>>laserOffset;
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    if(laser.setAngularOffset(laserOffset)) 
        cout<<"Laser offset is set to "<<laserOffset<<" degrees"<<endl;

    else cout<<"Invalid, default 0 degree offset will be used."<<endl;
    cout<<"laser X: "<<endl;
    cin>>laserX;
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    cout<<"laser Y: "<<endl;
    cin>>laserY;
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    //laser.setAngularResolution(30);
    laser.setSensorPose({laserX,laserY,laser.getAngularOffset()});

    cout<<"Laser Specifications:"<<endl;
    cout<<"Laser X :"<<laser.getSensorPose().x<<endl;
    cout<<"Laser Y : "<<laser.getSensorPose().y<<endl;
    cout<<"Laser offset:  "<<laser.getSensorPose().theta<<" degrees"<<endl;
    // else cout<<"Invalid, default 0 degree offset will be used."<<endl;
    cout<<"Laser Model: "<<laser.getModel()<<endl;
    cout<<"Laser Field of view: "<<laser.getFieldOfView()<<" degrees"<<endl;
    cout<<"Laser Maximum distance: "<<laser.getMaxRange()<<"m"<<endl;
    cout<<"Laser Minimum distance: "<<laser.getMinRange()<<"m"<<endl;
    cout<<"Laser Angular resolution: " << laser.getAngularResolution() << " degree" << endl;
    cout<<"---------------------------"<<endl;

    cout<<"Please insert laser pose for first sonar: "<<endl;
    cout<<"sonar1  offset: "<<endl;
    cin>>sonar1Offset;
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    cout<<"sonar1  X: "<<endl;
    cin>>sonar1X;
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    cout<<"sonar1  Y: "<<endl;
    cin>>sonar1Y;
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    if(sonar1.setAngularOffset(sonar1Offset)) cout<<"First Sonar offset is set to "<<sonar1Offset<<" degrees"<<endl;
    else cout<<"Invalid, default 0 degree offset will be used."<<endl;
    sonar1.setAngularResolution(0);
    sonar1.setSensorPose({sonar1X,sonar1Y,sonar1.getAngularOffset()});

    cout<<"Sonar1 specifications:"<<endl;
    cout<<"Sonar1 X :"<<sonar1.getSensorPose().x<<endl;
    cout<<"Sonar1 Y : "<<sonar1.getSensorPose().y<<endl;
    cout<<"Sonar1 offset:  "<<sonar1.getSensorPose().theta<<" degrees"<<endl;
    cout<<"Sonar1 Model: "<<sonar1.getModel()<<endl;
    cout<<"Sonar1 Field of view: "<<sonar1.getFieldOfView()<<" degrees"<<endl;
    cout<<"Sonar1 Maximum distance: "<<sonar1.getMaxRange()<<"m"<<endl;
    cout<<"Sonar1 Minimum distance: "<<sonar1.getMinRange()<<"m"<<endl;
    cout<<"Sonar1 Angular Offset: " <<sonar1.getAngularOffset() << " degree"<<endl;
    cout<<"---------------------------"<<endl;

    cout<<"Please input offset for second Sonar"<<endl;

    cout<<"sonar2  offset: "<<endl;
    cin>>sonar2Offset;
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    cout<<"sonar2  X: "<<endl;
    cin>>sonar2X;
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    cout<<"sonar2  Y: "<<endl;
    cin>>sonar2Y;
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    if(sonar2.setAngularOffset(sonar2Offset)) cout<<"Second Sonar offset is set to "<<sonar2Offset<<" degrees"<<endl;
    else cout<<"Your input was not sane, default 0 degree offset will be used."<<endl;

    sonar1.setAngularResolution(0);
    sonar1.setSensorPose({sonar2X,sonar2Y,sonar2.getAngularOffset()});

    cout<<"sonar2 specifications:"<<endl;
    cout<<"sonar2 X :"<<sonar2.getSensorPose().x<<endl;
    cout<<"sonar2 Y : "<<sonar2.getSensorPose().y<<endl;
    cout<<"sonar2 offset:  "<<sonar2.getSensorPose().theta<<" degrees"<<endl;
    cout<<"sonar2 Model: "<<sonar2.getModel()<<endl;
    cout<<"sonar2 Field of view: "<<sonar2.getFieldOfView()<<" degrees"<<endl;
    cout<<"sonar2 Maximum distance: "<<sonar2.getMaxRange()<<"m"<<endl;
    cout<<"sonar2 Minimum distance: "<<sonar2.getMinRange()<<"m"<<endl;
    cout<<"sonar2 Angular Offset: " <<sonar2.getAngularOffset() << " degree"<<endl;
    cout<<"---------------------------"<<endl;


    do
    {
        std::cout << "Input the number of cell for cell generation: " ;
        std::cin >> number_of_cell;
    } while(number_of_cell < 0);    //avoid invalid input

    do
    {
        std::cout << "Input Cell side: " ;
        std::cin >> cellside;
        cell.setSide(cellside);
    } while(cellside < 0);      //avoid invalid input

    for(int i = 0; i < number_of_cell; i++)
    {
        cells.push_back(new Cell());
    }

    vector<vector<double>> data;
    double area_data;
    vector<RangerInterface*> rangers;
    rangers.push_back(&laser);
    rangers.push_back(&sonar1);
    rangers.push_back(&sonar2);
    RangerFusion rangerfusion(rangers);
    rangerfusion.setCells(cells);
    cell.setSide(cellside);
    rangerfusion.setCellside(cellside);

    system("clear");
    cout<<"All set, Press ENTER to get the readings!"<<endl;
    cin.ignore();
    getchar();

    while(1)
    {
        counter+=1;
        int Unknown = 0, Free = 0, Occupy = 0;

        cout<<"Data#"<<counter<<endl;

        rangerfusion.grabAndFuseData();                         //check the cell of all the cell in range

        // print out the final out put for each sample
        cout<<"Raw Data:"<<endl;

        data = rangerfusion.printData();
        cout << "dataSize:"<<data.size()<<endl;
        for (int i = 0; i < data.size(); i++)
        {
            cout << "data ["<<i<<"] size: "<<data[i].size()<<endl;
            for (int j = 0; j < data[i].size(); j++)
                cout << data[i][j] << " ";
            cout << endl;
        }
        cout<<"-----------------"<<endl;

        cout << "Area covered by 2 sonar(m2): " << rangerfusion.getScanningArea() << endl;
        cout<<"-----------------"<<endl;

        std::cout << "Cell State:" << endl;

        for (auto it = cells.begin();it != cells.end(); it++ ) //Return the state for each cell
        {
            cout << (*it)->getState() << " ";
            if((*it)->getState()==0)
            {
                Unknown++;
            }
            if((*it)->getState()==1)
            {
                Free++;
            }
            if((*it)->getState()==-1)
            {
                Occupy++;
            }
        }

        cout<< endl <<"UNKNOWN = " << Unknown << endl;
        cout<<"Free = " << Free << endl;
        cout<< "OCCUPY = " << Occupy << endl;


        cout << endl;

        laser.delay();
    }

    return 0;
}
