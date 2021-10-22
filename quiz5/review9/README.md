Quiz 9
======

INFORMATION
------
- To develop the quiz 

symbolically link the [quiz9 folder](.) to your catkin_ws/src (ie if your quiz9 folder path is <git_repo_path>/quizzes/quiz9/ then execute:
```bash
cd ~/catkin_ws/src
ln -s <git_repo_path>/quizzes/quiz9/
```
- Compile with catkin_make
```bash
cd ~/catkin_ws
catkin_make
```

- You will need to run the a4_setup to enable testing your code
```bash
roslaunch a4_setup a4_setup.launch
```

- Sections for TASKS in the quiz have a `(//! @todo)` in code to enable identification

*For part A*

You are developing code that would respond to a service, with a bool value indicating the global coordinate trequested is within the OgMap.
Thereafter. every 5 seconds it will check if the global coordinate is within OgMap, and if it is via ROS_INFO state how many cells are between the robot and the global coordinate.


To find information about the service message `rossrv info a4_setup/RequestGoal`, which should display
```
geometry_msgs/Pose2D pose
  float64 x
  float64 y
  float64 theta
---
bool ack
```
This meand the pose is supplied as a request and the return value is a bool (the parts of the messages are seperated with `---`.

To run the executable for Quiz9a
```bash
rosrun quiz9a quiz9a-sample
```

After running `rosrun quiz_9a quiz_9a-sample` you wil be able to request a goal via the terminal, type below
```bash
rosservice call /check_goal
```
then hit *TAB* twice, will auto populate with entier message, edit x,y and hit enter to test code.
```bash
rosservice call /check_goal "x: 0.0
y: 0.0
theta: 0.0"
```

*For Part B*

```
- When running your package for Quiz6b (substitute XXX for name of node)
```bash
rosrun quiz9b quiz9b-XXX
```

To test task 1-3 run `rosrun quiz_9b quiz_9b-generate_og`, To test task 4-5 run `rosrun quiz_9b quiz_9b-analyse_laser`

Part A
------
1) QUESTION: Look at the code in [folder a](./a), what is the name of the package?

    - "quiz_9a"

2) TASK: In the code [seperateThread](./a/src/sample.cpp) "ros::Rate rate_limiter(XXX)" located in [sample.cpp](./a/sample.cpp) achieve running thread every 5 seconds.

3) QUESTION: In the code [requestGoal](./a/src/sample.cpp) we utilise a RequestGoal service. In what facility is it: (A) offer a service (incoming - recieve request) or (B) client (outgoing - makes request).  

    - (A)

4) TASK: In the code [requestGoal](./a/src/sample.cpp) augument the RequestGoal service to return true if the global coordinate supplied in the requst is within current OgMap (just needs to be within the map - either free / occupied or unknown).

5) TASK: In the code [seperateThread](./a/src/sample.cpp), if a goal has been requested, print the number of cells between the robot pose and the global coordinate requested. HINT: Check if the coordinate is on OgMap, and convert it to a local robot coordinate here, and determine the number of cells (we have the resolution that should assist in counting the distance).

Part B
------
1) QUESTION: In the [OccupancyGridMap message](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html), what does the value of -1 in the GridMap data indicate?
    - unknown.

2) TASK: In [mapCallback of PfmsSample in b/src/generate_ogmap.cpp](./b/src/generate_ogmap.cpp) count and display the number of uknown cells.

3) TASK: In [mapCallback of PfmsSample in b/src/generate_ogmap.cpp](./b/src/generate_ogmap.cpp) publish an augument new OgMap (newGrid) that has all unknown cells converted to 50% free (value of 50).

4) TASK: In [laserCallback of PfmsSample in ./b/src/analyse_laser.cpp](./b/src/analyse_laser.cpp), determine the number of high intensity readings

5) TASK: In [laserCallback of PfmsSample in ./b/src/analyse_laser.cpp](./b/src/analyse_laser.cpp), find a group of high intensity readings with neighbours within 0.3m of each other, + print the x,y location  of the first point and last point in this group of points
