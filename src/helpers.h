#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <array>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// for convenience
using std::string;
using std::vector;
using std::array;

using Eigen::MatrixXd;
using Eigen::VectorXd;
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
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

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

struct MapWaypoints{
  vector<double> x;
  vector<double> y;
  vector<double> s;
  vector<double> dx;
  vector<double> dy;
};

struct LocalizationData{
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
};

struct SensorFusionData{
  double v;
  double s;
  double d;
};

struct PreviousPath{
  vector<double> x;
  vector<double> y;
  unsigned int size;
};

void EvaluateFusionData(const LocalizationData& car, const SensorFusionData& other_car, array<bool, 3>& too_close, array<double, 3>& LaneSpeed, int lane){
  
  // if car is in my lane
  if( (other_car.d > lane) && (other_car.d < (lane+1.0)) ){
    // if speed is smaller than mine, go slower
    if ((other_car.s > car.s) && ((other_car.s - car.s) < 30.0)){
      too_close[lane] = true;
      if(other_car.v < LaneSpeed[lane]) LaneSpeed[lane] = other_car.v;
    }
  }
  //car is in lane left
  else if( (other_car.d > (lane-1.0)) && (other_car.d < lane) ){
    if ((other_car.s > (car.s - 5.0)) && ((other_car.s - car.s) < 30.0)){
      too_close[lane-1] = true;
      if(other_car.v < LaneSpeed[lane-1]) LaneSpeed[lane-1] = other_car.v;
    }
  }
  //car is in the lane right
  else if( (other_car.d > (lane+1.0)) && (other_car.d < (lane+2.0)) ){
    if ((other_car.s > (car.s - 5.0)) && ((other_car.s - car.s) < 30.0)){
      too_close[lane+1] = true;
      if(other_car.v < LaneSpeed[lane+1]) LaneSpeed[lane+1] = other_car.v;
    }
  }
}

class CoordinateTransform{
  const double m_x, m_y, m_yaw;
  public:
  explicit CoordinateTransform(const double& x, const double& y, const double& yaw) : m_x(x), m_y(y), m_yaw(yaw) {}

  void ToVehicleCoord(double& x, double& y) const {
    const double x_ = x - m_x;
    const double y_ = y - m_y;

    x = x_*cos(-m_yaw) - y_*sin(-m_yaw);
    y = x_*sin(-m_yaw) + y_*cos(-m_yaw);
  }

  void ToGlobalCoord(double& x, double& y) const {
    const double x_ = x*cos(m_yaw) - y*sin(m_yaw);
    const double y_ = x*sin(m_yaw) + y*cos(m_yaw);

    x = x_ + m_x;
    y = y_ + m_y;
  }
};

void StartingPoints_Spline(const LocalizationData& car, const PreviousPath& prev, vector<double>& ptsx, vector<double>& ptsy){
  if (prev.size < 2){
    ptsx.push_back(car.x - cos(car.yaw));
    ptsx.push_back(car.x);

    ptsy.push_back(car.y - sin(car.yaw));
    ptsy.push_back(car.y);
  }
  else{
    ptsx.push_back(prev.x[prev.size-2]);
    ptsx.push_back(prev.x[prev.size-1]);

    ptsy.push_back(prev.y[prev.size-2]);
    ptsy.push_back(prev.y[prev.size-1]);
  }
}

void EndPoints_Spline(const LocalizationData& car, const double& lane, const MapWaypoints& map_waypoints, vector<double>& ptsx, vector<double>& ptsy){
  const double lane_d = 2.0 + 4.0*lane;

  auto next_wp0 = getXY(car.s+30.0, lane_d, map_waypoints.s, map_waypoints.x, map_waypoints.y);
  auto next_wp1 = getXY(car.s+60.0, lane_d, map_waypoints.s, map_waypoints.x, map_waypoints.y);
  auto next_wp2 = getXY(car.s+90.0, lane_d, map_waypoints.s, map_waypoints.x, map_waypoints.y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
}

int ConsiderLaneChange(const array<bool, 3>& too_close, int lane){
  if (lane == 1) {
    if      (!too_close[lane-1]) lane = 0;
    else if (!too_close[lane+1]) lane = 2;
  }
  else if ( (lane == 2) && !too_close[lane-1] ) lane = 1;
  else if ( (lane == 0) && !too_close[lane+1] ) lane = 1;
  return lane;
}

#endif  // HELPERS_H