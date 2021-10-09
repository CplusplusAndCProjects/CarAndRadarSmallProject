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
    int resolution = 10, offset1, offset2, offset3;   //Variable to hold the user input
    int number_of_cell;
    double cellside;
    double x, y;
    std::vector<Cell*> cells;
    Cell cell;

    cout<<"Asignment 2 - Programming for Mechatronics System" << endl;
    cout<<"13432190 - Trong Duy Bui" << endl;

    cout<<"Please insert angular resolution for the laser, it should be either 10 or 30 degrees:"<<endl;
    cin>>resolution;
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    if(laser.setAngularResolution(resolution)) cout<<"Your laser is set to "<<resolution<<" degrees angular resolution."<<endl;
    else cout<<"Invalid input, default 10 degrees angular resolution will be used."<<endl;
    cout<<"Please insert angular offset for the laser: "<<endl;
    cin>>offset3;
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    if(laser.setAngularOffset(offset3)) cout<<"Laser offset is set to "<<offset3<<" degrees"<<endl;
    else cout<<"Invalid, default 0 degree offset will be used."<<endl;
    laser.setAngularOffset(offset3);
    cout<<"Laser Specifications:"<<endl;
    cout<<"Model: "<<laser.getModel()<<endl;
    cout<<"Field of view: "<<laser.getFieldOfView()<<" degrees"<<endl;
    cout<<"Maximum distance: "<<laser.getMaxRange()<<"m"<<endl;
    cout<<"Minimum distance: "<<laser.getMinRange()<<"m"<<endl;
    cout<<"Angular resolution: " << laser.getAngularResolution() << " degree" << endl;
    cout<<"---------------------------"<<endl;

    cout<<"Please input offset for first sonar"<<endl;
    cin>>offset1;
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    if(sonar1.setAngularOffset(offset1)) cout<<"First Sonar offset is set to "<<offset1<<" degrees"<<endl;
    else cout<<"Invalid, default 0 degree offset will be used."<<endl;
    sonar1.setAngularResolution(0);
    cout<<"Sonar specifications:"<<endl;
    cout<<"Model: "<<sonar1.getModel()<<endl;
    cout<<"Field of view: "<<sonar1.getFieldOfView()<<" degrees"<<endl;
    cout<<"Maximum distance: "<<sonar1.getMaxRange()<<"m"<<endl;
    cout<<"Minimum distance: "<<sonar1.getMinRange()<<"m"<<endl;
    cout<<"Angular Offset: " <<sonar1.getAngularOffset() << " degree"<<endl;
    cout<<"---------------------------"<<endl;

    cout<<"Please input offset for second Sonar"<<endl;
    cin>>offset2;
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    if(sonar2.setAngularOffset(offset2)) cout<<"Second Sonar offset is set to "<<offset2<<" degrees"<<endl;
    else cout<<"Your input was not sane, default 0 degree offset will be used."<<endl;
    sonar2.setAngularResolution(0);
    cout<<"----------------------------"<<endl;
    cout<<"Sonar specifications:"<<endl;
    cout<<"Model: "<<sonar2.getModel()<<endl;
    cout<<"Field of view: "<<sonar2.getFieldOfView()<<" degrees"<<endl;
    cout<<"Maximum distance: "<<sonar2.getMaxRange()<<"m"<<endl;
    cout<<"Minimum distance: "<<sonar2.getMinRange()<<"m"<<endl;
    cout<<"Angular Offset: " <<sonar2.getAngularOffset() << " degree"<<endl;
    cout<<"----------------------"<<endl;

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
    rangers.push_back(&sonar1);
    rangers.push_back(&laser);
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

        data = rangerfusion.print_Data();

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
