#include "analysis.h"
#include "tf.h"
#include "tf2.h"

using std::vector;
using std::pair;
using geometry_msgs::Point;

Analysis::Analysis(std::vector<Point> goals) :
    goals_(goals)
{

}


//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
vector<double> Analysis::timeToImpact(Pose origin){

    //The consts you will need are
    //Display::G_MAX
    //Display::V_TERM
    //Display::V_MAX
    vector<double> times;
    for (size_t i = 0; i < goals_.size(); i++)
    {
        /* code */
        RangeBearingStamped rangeBearingStamped = global2local(goals_.at(i), origin);

        double bearing_drgrees = rangeBearingStamped.bearing *180/M_PI;

        if(bearing_drgrees < -180 || bearing_drgrees > 180)
            bearing_drgrees = 360 - abs(bearing_drgrees);
        rangeBearingStamped.bearing = bearing_drgrees * M_PI/180;

        double Omega_max = Simulator::G_MAX * 9.81 / Simulator::V_TERM; 
        double time = rangeBearingStamped.range/Simulator::V_MAX + rangeBearingStamped.bearing/Omega_max;
        times.push_back(time);
    }

    return times;
}

//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
AdjacencyList Analysis::exportGraph(){

    cout<<"Function exportGraph"<<endl;
    AdjacencyList graph;
    vector<vector<EdgeInfo> > adjacencyListForMultiNode;
    unsigned int id = 0;
    for (size_t i = 0; i < goals_.size(); i++)
    {
        Point goals_i = goals_.at(i);
        vector<EdgeInfo>  adjacencyListForOneNode;
        for (size_t j = 0; j < goals_.size(); j++)
        {
            Point goals_j = goals_.at(j);
            EdgeInfo edgeInfo ;
            if(i == j)
            {
                //  edgeInfo.first = i;
                //  adjacencyListForOneNode.push_back(edgeInfo);
                continue;

            }
            double euclideanDistance = sqrt(pow((goals_i.x - goals_j.x),2) + pow((goals_i.y - goals_j.y),2));
            edgeInfo = EdgeInfo(euclideanDistance, j);
            adjacencyListForOneNode.push_back(edgeInfo);
            id++;
            cout<<"Function exportGraph id = "<<id<<endl;

            /* code */
        }

        cout<<"Function finish for loop id = "<<id<<endl;
        adjacencyListForMultiNode.push_back(adjacencyListForOneNode);
        graph = AdjacencyList(adjacencyListForMultiNode);
        /* code */
    }

    
    //std::make_pair(odom.at(i), i));
 

    return graph;
}