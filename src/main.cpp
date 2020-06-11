#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  static double set_vel = 0;

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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;



          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          int prev_size = previous_path_x.size();
          vector<double> anchor_x;
          vector<double> anchor_y;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          double max_vel = 49.5;
          int lane = 1;
          int lane_width = 4;
          double anchor_spacing = 30.0;
          double dist_inc = 0.3; // spacing between the waypoints


          // Avoid collisions
          // 1. Iterate through all the cars and check if they are in our lane
          // 2. For the cars in our lane: check for the time gap - if the time gap is smaller than expected, adapt the target speed
          // 3. Feed the target speed in to the waypoint generator

          int idx = 99;
          double distance2collision = 9999.9;
          double ttc = 99.9;
          for (int i = 0; i < sensor_fusion.size(); i++){
        	  // Check for all the objects within the ego lane
        	  if ((sensor_fusion[i][6] >= lane_width * lane) && (sensor_fusion[i][6] <= (lane + 1) * lane_width)){
        		  double target_vx = sensor_fusion[i][3];
        		  double target_vy = sensor_fusion[i][4];
        		  double target_s = sensor_fusion[i][5];
        		  double delta_s = target_s - car_s;
        		  if ((distance2collision > delta_s) && (delta_s > 0)){
        			  idx = i;
        			  distance2collision = delta_s;
        			  std::cout<< "Delta distance: " << delta_s << std::endl;
        		  }
        	  }
          }
          std::cout<< "Distance2collision: " << distance2collision << std::endl;

          if (idx != 99){
           	  // Calculate the time to collision metric
        	  double target_vx = sensor_fusion[idx][3];
        	  double target_vy = sensor_fusion[idx][4];
              double target_s = sensor_fusion[idx][5];
        	  double target_speed = sqrt(target_vx * target_vx + target_vy * target_vy)/2.24;
        	  // Calculate the distance between the target vehicle and the last point of the planned path
        	  double s_diff = target_s - end_path_s;
        	  // if the target car is far away and the ego car is too slow, set the time to collision to a large value
        	  if ((s_diff > 50) && (car_speed / 2.24 - target_speed < -5.0)){
        		  ttc = 999.9;
        	  }
        	  // if the target car is close enough and speed
        	  else{
        		  ttc = s_diff / (car_speed / 2.24 - target_speed);
        	  }

        	  std::cout<< "TTC: " << ttc << std::endl;
          }
          // If TTC is too small, slow down
          if (ttc <= 1.5){
        	  set_vel += -0.3;
        	  std::cout<< "Decrease the velocity: " << std::endl;
          }
          else{
        	  if ((max_vel - set_vel > 0.0) && (set_vel < 49.5)) {
        		  set_vel += 0.3;
        		  std::cout<< "Increase the velocity: " << std::endl;
        	  }
          }

          std::cout<< "Set velocity: " << set_vel << std::endl;


          // if the size of the previous path is too small, use the current position of the car and project it one step to the past based on the current orientation
          if (prev_size < 2){
        	  double prev_car_x = car_x - cos(car_yaw);
        	  double prev_car_y = car_y - sin(car_yaw);

        	  anchor_x.push_back(prev_car_x);
        	  anchor_x.push_back(car_x);

        	  anchor_y.push_back(prev_car_y);
        	  anchor_y.push_back(car_y);
          }

          // if the previous path has enough points, use the previous two points
          else{

        	  ref_x = previous_path_x[prev_size - 1];
        	  ref_y = previous_path_y[prev_size - 1];

        	  double refpoint_x_2 = previous_path_x[prev_size - 2];
        	  double refpoint_y_2 = previous_path_y[prev_size - 2];
        	  ref_yaw = atan2(ref_y - refpoint_y_2, ref_x - refpoint_x_2);

        	  anchor_x.push_back(refpoint_x_2);
        	  anchor_x.push_back(ref_x);

        	  anchor_y.push_back(refpoint_y_2);
        	  anchor_y.push_back(ref_y);
          }

          // Define anchor points in front of the vehicle in Cartesian coordinate system for further spline estimation
          vector<double> xy_preview_anchor1 = getXY(car_s + 1 * anchor_spacing, lane_width/2 + lane_width * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> xy_preview_anchor2 = getXY(car_s + 2 * anchor_spacing, lane_width/2 + lane_width * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> xy_preview_anchor3 = getXY(car_s + 3 * anchor_spacing, lane_width/2 + lane_width * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          anchor_x.push_back(xy_preview_anchor1[0]);
          anchor_x.push_back(xy_preview_anchor2[0]);
          anchor_x.push_back(xy_preview_anchor3[0]);

          anchor_y.push_back(xy_preview_anchor1[1]);
          anchor_y.push_back(xy_preview_anchor2[1]);
          anchor_y.push_back(xy_preview_anchor3[1]);

          for (int i = 0; i < anchor_x.size(); i++){

        	  // transform the car reference angle to the ego car coordinate system (reference angle will be 0 degrees then)
        	  double shift_x = anchor_x[i] - ref_x;
        	  double shift_y = anchor_y[i] - ref_y;

        	  anchor_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        	  anchor_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          tk::spline s;

          s.set_points(anchor_x, anchor_y);


          vector<double> interpolated_points_x;
          vector<double> interpolated_points_y;

          for (int i = 0; i < previous_path_x.size(); i++){
        	  interpolated_points_x.push_back(previous_path_x[i]);
        	  interpolated_points_y.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x);
          //std::cout << "target y: " << target_y << std::endl;
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;

          for (int i = 1; i <= 50 - previous_path_x.size(); i++){
        	  double N = (target_dist / (0.02 * set_vel/2.24));
        	  double x_point = x_add_on + target_x/N;
        	  double y_point = s(x_point);
        	  x_add_on = x_point;

        	  double x_ref = x_point;
        	  double y_ref = y_point;

        	  // Rotating back
        	  x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        	  y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        	  x_point += ref_x;
        	  y_point += ref_y;

        	  interpolated_points_x.push_back(x_point);
        	  interpolated_points_y.push_back(y_point);
          }

          msgJson["next_x"] = interpolated_points_x;
          msgJson["next_y"] = interpolated_points_y;


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
