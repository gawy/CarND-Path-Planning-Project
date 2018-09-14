//
// Created by Andrii Garkavyi on 9/12/18.
//

#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <vector>
#include <string>
#include <map>

using std::vector;

enum class BhState {KL, LCL, LCR};

const double LANE_WIDTH = 4.0; //meters
const int LANE_COUNT = 3;
const double TIME_INC = 0.02; //sec. How often car expects points
const double IDEAL_VELOCITY = 49.5 * 1609.34 / 3600; //50mph to m/s

vector<vector<double>> planTrajectory(double car_x, double car_y, double car_yaw, int lane, double target_velocity,
                                      vector<double> &previous_path_x, vector<double> &previous_path_y,
                                      vector<double> &map_waypoints_x, vector<double> &map_waypoints_y,
                                      vector<double> &map_waypoints_s);

vector<BhState> getNextStates(BhState current_state, int car_lane);
std::map<int, double> getLaneAvgSpeed(vector<vector<double >> &sensor_fusion, double car_s);
double costSpeed(int lane, double car_s, vector<vector<double >> &sensor_fusion);


double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

#endif //PATH_PLANNING_PLANNER_H
