#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <array>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::array;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  MapWaypoints map_waypoints;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints.x.push_back(x);
    map_waypoints.y.push_back(y);
    map_waypoints.s.push_back(s);
    map_waypoints.dx.push_back(d_x);
    map_waypoints.dy.push_back(d_y);
  }

  // max speed = 22.3 m/s -> 0.446 distance increment
  // max accel = 10 m/s^2 -> 0.2 speed increment
    
  constexpr int pts_in_traj{50};
  constexpr double s_to_pt{0.02};
  constexpr double max_speed{22.3};
  constexpr double max_accel{9.5};
  constexpr double max_jerk{9.5};

  constexpr double max_pt_distance_in_cycle{max_speed * s_to_pt};
  constexpr double max_speed_change_in_cycle{max_accel * s_to_pt};

  double ref_vel{0.0};
  double lane = 1.0;

  h.onMessage([&map_waypoints, &ref_vel, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          LocalizationData car;
          car.x = j[1]["x"];
          car.y = j[1]["y"];
          car.s = j[1]["s"];
          car.d = j[1]["d"];
          car.yaw = j[1]["yaw"];
          car.speed = j[1]["speed"];

          // Previous path data given to the Planner
          PreviousPath prev;
          for (const auto& elem : j[1]["previous_path_x"]) prev.x.push_back(elem);
          for (const auto& elem : j[1]["previous_path_y"]) prev.y.push_back(elem);
          prev.size = prev.x.size();

          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          if(prev.size > 0) car.s = end_path_s;

          array<double, 3> LaneSpeed{max_speed, max_speed, max_speed};
          array<bool, 3> too_close{false, false, false};

          for(int i=0; i<sensor_fusion.size(); ++i){
            SensorFusionData other_car;
            const double vx = sensor_fusion[i][3];
            const double vy = sensor_fusion[i][4];
            other_car.v = sqrt(vx*vx+vy*vy);

            other_car.s = sensor_fusion[i][5];
            other_car.s += s_to_pt * other_car.v * prev.size;

            other_car.d = sensor_fusion[i][6];
            other_car.d *= 0.25;

            EvaluateFusionData(car, other_car, too_close, LaneSpeed, lane);
          }

         if(too_close[lane]) lane = ConsiderLaneChange(too_close, lane);

          if (too_close[lane] && (ref_vel > LaneSpeed[lane])) ref_vel -= max_speed_change_in_cycle;
          else if(!too_close[lane] && (ref_vel < (max_speed - max_speed_change_in_cycle))) ref_vel += max_speed_change_in_cycle;

          vector<double> ptsx;
          vector<double> ptsy;

          StartingPoints_Spline(car, prev, ptsx, ptsy);
          EndPoints_Spline(car, lane, map_waypoints, ptsx, ptsy);

          const double ref_yaw = atan2(ptsy[1] - ptsy[0], ptsx[1] - ptsx[0]);
          const CoordinateTransform transform(ptsx[1], ptsy[1], ref_yaw);

          for(int i=0; i < ptsx.size(); ++i) 
          transform.ToVehicleCoord(ptsx[i], ptsy[i]);

          tk::spline s;
          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals{std::move(prev.x)};
          vector<double> next_y_vals{std::move(prev.y)};

          constexpr double target_x{30.0};
          const double target_y = s(target_x);
          const double target_dist = sqrt(target_x*target_x + target_y*target_y);

          const double D = target_dist / (s_to_pt * ref_vel);
          double x_add_on = 0.0;

          for (int i=1; i<= pts_in_traj-prev.size; ++i){

            double x_point = x_add_on+target_x/D;
            double y_point = s(x_point);

            x_add_on = x_point;

            transform.ToGlobalCoord(x_point, y_point);

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}