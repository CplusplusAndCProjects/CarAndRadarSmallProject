#ifndef ANALYSIS_H
#define ANALYSIS_H

#include <vector>
#include <thread>
#include "types.h"
#include "display.h"
#include "tf2.h"
using geometry_msgs::Pose;
using geometry_msgs::Point;
using std::pair;

typedef pair<double, unsigned int> EdgeInfo;
typedef vector<vector<EdgeInfo> > AdjacencyList;

using namespace tf2;
using namespace std;
class Analysis
{
public:
    /**
     * @brief Constructor for class
     * @param vector of Point(s) thaty are goals
     */
    Analysis(std::vector<Point> goals);

    /**
     * @brief Determines time to impact (time to reach each goal)
     * @note To reach any of the goals from the origin you can either turn (at max permisable angular velocity
     * OMEGA MAX while having lowest liner velocity V_TERM) or go straight (at maximum possible linear
     * velocity V_MAX. Whilst obeying OMEGA*V<= 6G.</br>
     * Therefore, for each goal the computation for time to impact has to take into account the distance
     * and angle for each goal from the origin pose.
     *
     * @param origin for the search
     * @return time to impact for each goal(s) (this vector has same size as the poses supplied)
     * each element is time to impact for the respective goal
     */
    std::vector<double> timeToImpact(Pose origin);


    /**
     * @brief
     * Creates a graph of all goals, all nodes are interconnected
     * if number of nodes is N then each node is connected to N-1 nodes.
     * Store graph using the #AdjacencyList.
     *
     * In the graph:
     * 1) the outer vector contains an entry for all nodes // like our Week 5 BFS/DFS example
     * 2) the inner vector has edges a pair, where the pairs contains
     *  * a metric (we will use Euclidian distance between the nodes) - as fisrt element
     *  * the node id the edge is connecting to - as second element
     * @return An adjacencylist interconecting all nodes
     */
    AdjacencyList exportGraph();

private:
    std::vector<Point> goals_;
};

#endif // ANALYSIS_H
