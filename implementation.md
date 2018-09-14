# Behavior planning project implementation

### Approach

All behavior planning logic is distributed between main.cpp and planner.cpp.

Video of the car running full track [https://youtu.be/0XWXdWdSofw](https://youtu.be/0XWXdWdSofw)

Key methods in planner.cpp are also covered with basic unit tests which greatly simplifies validation of written code
(no need to run simulator each time and do painful debugging).

main.cpp will run behavior planner and trajectory planning in the same cycle and does not distibute those in time.

The algorithm is next:
* identify distance to closest car ahead
* accelearate or decelerate based on distance to obstacle
* generate list of possible state
* calculate cost for each state change and select cheapest
* generate trajectory for a new state

This implementation does not account for detailed trajectory in cost functions and has its own problems.

### Trajectory generation
In order to simplify logic with the curve generation I have used a spline library recommended in the course.

Function responsible for generating trajectory points. planner.cpp:124 
``` 
vector<vector<double>> planTrajectory(double car_x, double car_y, double car_yaw, int lane, double target_velocity,
                                      vector<double> &previous_path_x, vector<double> &previous_path_y,
                                      vector<double> &map_waypoints_x, vector<double> &map_waypoints_y,
                                      vector<double> &map_waypoints_s)
``` 
                                         
Each new iteration starts with adding anchor points for spline. First 2 are usually from the previously generated set 
or in case of the first run - current position and calculated prior position of the car based on its yaw angle.

After that 3 more anchor points are added. Those are distributed 30m from each other.

All following manipulation is done in car coordinate system. So as soon as we add all anchor points they are converted 
to car coordinates.

Final waypoints are majorly formed from previously generated path so only few new points are added to the path 
in the next iteration.

Observed problems that should be fixed: 
* when car stops in traffic spline library would cause an exception and crash so handling for such corner case should be added.
* accelearation and deceleration are done in a way explained in project walkthrough and is not optimal.
Ideally it would be to incorporate velocity change at each newly generated interval whic could lead to a more smooth
driving.

### Building state machine

In order to navigate car in a freeway conditions a simple Finate State Machine was implemented.
It has only 3 states: Keep Lane, Change Lane Right, Change Lane Left. 
For this project it was decided not to add more and try less states which proved it self sufficiently good 
but definitely not ideal.

FSM provides input to trajectory generator and right now it just changes intended lane value.
Trajectory generator will automatically adjust spline to go to an intended lane.

Obviously such simple state implementation does not cover various planning cases that could lead to more optimal 
navigation. For example:
1) car may stuck in traffic when in lef/right most lane and near by is also packed but most distant is free. 
Current implementation does not account for such cases but those can be added by tweaking cost functions. 

### Selecting cost functions

Majority of cost functions are implemented in planner.cpp Lines 91-121

Current implementation rewards for:
* going at maximum allowed speed;
* not doing consequent lane changes - large d changes for a car
* choosing lane that would allow to go further before closest obstacle. This cost function also accounts for cars 
that are too close and so also plays as kind of collision prevention system when doing lane change maneuvers 

Combining cost functions together and adding weights. main.cpp:200
```
double lateral_disp_cost = (new_lane * LANE_WIDTH + 0.5 * LANE_WIDTH - car_d) / LANE_WIDTH;
lateral_disp_cost = tanh(0.3*lateral_disp_cost*lateral_disp_cost*lateral_disp_cost);

double c = costSpeed(new_lane, car_s, sensor_fusion) + 2 * lateral_disp_cost
           + 1.5 * costDistance(new_lane, car_s, sensor_fusion);
```

Issues:
- proper collision avoidance needs to be impelemnted as car potentialy can change lane in front of a fast moving car
