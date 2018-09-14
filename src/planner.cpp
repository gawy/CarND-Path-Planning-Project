//
// Created by Andrii Garkavyi on 9/12/18.
//

#include "planner.h"
#include <iostream>
#include <math.h>
#include "spline.h"
#include <limits>


using namespace std;

//enumeration types (both scoped and unscoped) can have overloaded operators
std::ostream& operator<<(std::ostream& os, BhState s)
{
  switch(s)
  {
    case BhState::KL   : os << "KL";    break;
    case BhState::LCL: os << "LCL"; break;
    case BhState::LCR : os << "LCR";  break;
    default    : os.setstate(std::ios_base::failbit);
  }
  return os;
}

// Returns list of states that are possible for current state
// This is essentially a definition of finite state machine
vector<BhState> getNextStates(const BhState current_state, int car_lane) {
  vector<BhState> states ;

  switch (current_state) {

    case BhState::LCL:
    case BhState::LCR:
      states.push_back(BhState::KL);
      break;
    case BhState::KL:
      states.push_back(BhState::KL);
      if (car_lane < LANE_COUNT - 1) { //not most right
        states.push_back(BhState::LCR);
      }
      if (car_lane > 0) { //we are not in the most left
        states.push_back(BhState::LCL);
      }
      // no default in case I forget to ammend conditions here when new state is introduced
  }

  return states;
}

// Return average lane speed based on the sensor fusion data
// Function will take all cars ahead of car_s from sensor fusion array and will average their speed in each lane.
//
map<int, double> getLaneAvgSpeed(vector<vector<double >> &sensor_fusion, double car_s) {
  map<int, double> lane_average;
  map<int, int> lane_cnt;

  for (int i = 0; i < sensor_fusion.size(); ++i) {
    auto drone = sensor_fusion[i];
    double drone_s = drone[5];
    if (drone_s < car_s) { continue; }

    double drone_d = drone[6];
    int drone_lane = (int) (drone_d / LANE_WIDTH);

    double drone_vx = drone[3];
    double drone_vy = drone[4];
    double drone_velocity = sqrt(drone_vx*drone_vx + drone_vy*drone_vy);

    if (lane_average.find(drone_lane) == lane_average.end()) {
      lane_average[drone_lane] = drone_velocity;
      lane_cnt[drone_lane] = 1;
    } else {
      lane_average[drone_lane] += drone_velocity;
      lane_cnt[drone_lane] += 1;
    }
  }

//  cout << "Lane average speed: ";
  for(auto k : lane_average) {
    lane_average[k.first] /= lane_cnt[k.first];
//    cout << "{"<< k.first << "=" << lane_average[k.first] << "}";
  }
//  cout << endl;

  return lane_average;
}

// Calculate cost of going to specific lane based on average speed in target lane
double costSpeed(int lane, double car_s, vector<vector<double >> &sensor_fusion) {
  map<int, double> avg_speed = getLaneAvgSpeed(sensor_fusion, car_s);

  double speed_delta = avg_speed[lane] ? IDEAL_VELOCITY - avg_speed[lane]: 0.0;

  return 1 - exp(-2*speed_delta/IDEAL_VELOCITY);
}

// Calculate cost of going to specific lane based on how far we can get till next drone in target lane
double costDistance(int lane, double car_s, vector<vector<double >> &sensor_fusion) {
  double distance_in_lane = 999.9; //just a large number that is suposedly is bigger than any object in sensor fusion array
  double s_safe_delta = 5.0;

  for (int i = 0; i < sensor_fusion.size(); ++i) {
    auto drone = sensor_fusion[i];
    double drone_s = drone[5];

    double drone_d = drone[6];
    int drone_lane = (int) (drone_d / LANE_WIDTH);

    if (drone_s + s_safe_delta < car_s || drone_lane != lane) { continue; } //skip cars behind

    double distance = abs(drone_s - car_s); // this will be similar to drone too close ahead
    if (distance < distance_in_lane) {
      distance_in_lane = distance;
    }

  }

  return 1 - exp(-30.0/distance_in_lane);
}


vector<vector<double>> planTrajectory(double car_x, double car_y, double car_yaw, int lane, double target_velocity,
                                      vector<double> &previous_path_x, vector<double> &previous_path_y,
                                      vector<double> &map_waypoints_x, vector<double> &map_waypoints_y,
                                      vector<double> &map_waypoints_s) {
  vector<double> next_x_vals;
  vector<double> next_y_vals;


  vector<double> pts_x {};
  vector<double> pts_y {};
  double ref_yaw = car_yaw;
  if (!previous_path_x.empty()) {
    int prev_pts_cnt = 3;
    cout << "Preparing pts for spline cnt=" << prev_pts_cnt << endl;
    for (auto i = previous_path_x.size() - prev_pts_cnt; i < previous_path_x.size(); ++i) {
      pts_x.push_back(previous_path_x[i]);
      pts_y.push_back(previous_path_y[i]);
    }

    unsigned long pts_size = pts_x.size();
    ref_yaw = atan2(pts_y[pts_size-1] - pts_y[pts_size-2], pts_x[pts_size-1] - pts_x[pts_size-2]);
  } else {
    pts_x.push_back(car_x - cos(car_yaw));
    pts_y.push_back(car_y - sin(car_yaw));

    pts_x.push_back(car_x);
    pts_y.push_back(car_y);
    ref_yaw = car_yaw;
  }

  double ref_x = pts_x[pts_x.size() - 1];
  double ref_y = pts_y[pts_y.size() - 1];

  double next_d = lane * LANE_WIDTH + 0.5 * LANE_WIDTH;
  vector<double> ref_frenet = getFrenet(ref_x, ref_y, car_yaw, map_waypoints_x, map_waypoints_y);

  for (int i = 1; i < 4; i++) {
    double next_s = ref_frenet[0] + i * 30;
    vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    pts_x.push_back(xy[0]);
    pts_y.push_back(xy[1]);
  }

  cout << "Real coord spline: ";
  for (int i = 0; i < pts_x.size(); ++i) {
    cout << "(" << pts_x[i] << "," << pts_y[i] << ")";
  }
  cout << endl;


  //show me pre-spline pts
//          std::cout << "Car coord spline, (" << ref_x << "," << ref_y << ") =(";
  for (int k = 0; k < pts_x.size(); ++k) {
    double x_new = (pts_x[k] - ref_x)*cos(-ref_yaw) - (pts_y[k] - ref_y)*sin(-ref_yaw);
    double y_new = (pts_x[k] - ref_x)*sin(-ref_yaw) + (pts_y[k] - ref_y)*cos(-ref_yaw);

    pts_x[k] = x_new;
    pts_y[k] = y_new;
//            std::cout << "(" << pts_x[k] << "," << pts_y[k] << ")";
  }
//          std::cout << ")" << std::endl;

  tk::spline s;
  s.set_points(pts_x, pts_y);

  //add previous points
//          std::cout << "Previous path: ";
  for (int i = 0; i < previous_path_x.size(); ++i) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);

//            std::cout << "(" << previous_path_x[i] << "," << previous_path_y[i] << ")";
  }
//          std::cout << std::endl;

  // add points based on spline
  double dst_x = 30;
  double dst_y = s(dst_x);
  double dst_dist = distance(0, 0,  dst_x, dst_y);
//          std::cout << "Heading to (car coords): (" << dst_x << "," << dst_y << ") distance=" << dst_dist << std::endl;

  unsigned long time_steps = 50 - previous_path_x.size();//(int) (dst_dist / TARGET_SPEED * TIME_INC);
  double N = dst_dist / (target_velocity * TIME_INC);
  double x_inc = dst_x / N;
  cout << "spline pts: inc=" << x_inc << ", dist=" << dst_dist << ", steps=" << time_steps <<
       ", N=" << N <<";" << endl << "[";
  for (int i = 1; i <= time_steps; ++i) {

    double pt_x = x_inc * i;
    double pt_y = s(pt_x);

//            std::cout << "s(" << pt_x << "," << pt_y << ")";
    double x_new = ref_x + (pt_x*cos(ref_yaw) - pt_y*sin(ref_yaw));
    double y_new = ref_y + (pt_x*sin(ref_yaw) + pt_y*cos(ref_yaw));

    next_x_vals.push_back(x_new);
    next_y_vals.push_back(y_new);

    cout << "(" << x_new << "," << y_new << ") # ";
  }
  cout << "]" << endl;

  return {next_x_vals, next_y_vals};
}


// Convenience functions

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }