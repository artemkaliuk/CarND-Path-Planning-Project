#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "planner.hpp"
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
  static double set_vel = 0.0;

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

          /**2
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          double v_x = sensor_fusion[0][3];
          double v_y = sensor_fusion[0][4];
          double pos_s = sensor_fusion[0][5];
          double pos_d = sensor_fusion[0][6];

          int prev_size = previous_path_x.size();
          vector<double> anchor_x;
          vector<double> anchor_y;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          double safe_timegap = 2; // time gap between the ego and target vehicle

          double max_acc_m_s_2 = 10; // maximum acceleration in meters per second
          double max_acc_mphps_cycle = max_acc_m_s_2 * 2.2369 * 0.02 * 0.8; // 80% of the maximum acceleration in miles per hour per frame
          // Calculate the safe distance in case if the front vehicle applies maximum deceleration
          double max_vel_mph = 50.0;
          double max_vel = max_vel_mph * 1.609 / 3.6; // maximal allowed speed meters per second

          //double max_vel_target_final_m_s = max_vel - max_acc_s * safe_timegap;
          //double max_vel_difference = max_vel - (max_vel_target_final_m_s);
          double safe_dist = 30;

          double lane;
          double lane_neighb1;
          double lane_neighb2;
          double lane_width = 4.0;
          int target_lane_idx;
          double anchor_spacing = 50.0; // spacing between the anchor points of the spline for smoother trajectories
          double dist_inc = 0.3; // spacing between the waypoints
          //double max_acc = 10 * 0.016;

          // identify the ego vehicle's current lane
          if((car_d >= lane_identifiers::left * lane_width) && (car_d < (lane_identifiers::left + 1.0) * lane_width)){
        	  lane = 0.0;
        	  lane_neighb1 = 1.0;
        	  lane_neighb2 = 2.0;

          }
          else if((car_d >= lane_identifiers::middle * lane_width) && (car_d < (lane_identifiers::middle + 1.0) * lane_width)){
        	  lane = 1.0;
        	  lane_neighb1 = 0.0;
        	  lane_neighb2 = 2.0;
          }
          else{
        	  lane = 2.0;
        	  lane_neighb1 = 1.0;
        	  lane_neighb2 = 0.0;
          }
          // find the target vehicle
          Lane egoLane = Lane(lane, lane_width, car_s, car_d, car_speed, max_vel, sensor_fusion);
          Lane neighb1Lane = Lane(lane_neighb1, lane_width, car_s, car_d, car_speed, max_vel, sensor_fusion);
          Lane neighb2Lane = Lane(lane_neighb2, lane_width, car_s, car_d, car_speed, max_vel, sensor_fusion);

          std::cout << "set velocity: " << set_vel << std::endl;

          //if (((safe_dist + 15) > egoLane.target_s) && (egoLane.target_delta_v > max_acc_cycle) && (set_vel > 5.0)){
          if ((safe_dist > egoLane.target_s) || ((safe_dist + 5 > egoLane.target_s) && (egoLane.target_delta_v > max_acc_mphps_cycle))){
              set_vel -= max_acc_mphps_cycle;
        	  std::cout << "1 Target : " << egoLane.target_s << " Target delta: " << egoLane.target_delta_v << " max acc: " << max_acc_mphps_cycle << std::endl;
        	  std::cout << "Deceleration 1" << std::endl;
          }
/*          else if (((safe_dist + 15) > egoLane.target_s) && (egoLane.target_delta_v > max_acc_mphps_cycle)){
        	  set_vel -= max_acc_mphps_cycle*0.5;
        	  std::cout << "2 Target s1: " << egoLane.target_s << " Target delta: " << egoLane.target_delta_v << std::endl;
        	  std::cout << "Deceleration 2!!" << std::endl;
          }
          */
          else {
        	  std::cout << "Acceleration!!!" << std::endl;
        	  if (set_vel < (max_vel_mph - 0.5)){
            	  set_vel += max_acc_mphps_cycle;

        	  }
          }


          std::cout << "Target id: " << egoLane.target_id << "Target dist: " << egoLane.target_s << " Delta v: " << egoLane.target_delta_v << " Set speed: " << set_vel << std::endl;

          // maximum change of speed allowed between two frames
          //Lane egoLane = Lane(lane, lane_width, car_s, car_speed, max_vel);
          //int neighborLane1_id = (lane + 1) % 3;
          //Lane neighborLane1 = Lane(neighborLane1_id, lane_width, car_s, car_speed, max_vel);
          //int neigh borLane2_id = (lane + 2) % 3;
          //Lane neighborLane2 = Lane(neighborLane2_id, lane_width, car_s, car_speed, max_vel);



          //egoLane.getFront(sensor_fusion, egoLane.lane_width, egoLane.ego_s, egoLane.ego_speed, &egoLane.id_front, &egoLane.distance2collision_front, &egoLane.target_v_front, &egoLane.ttc_front, &egoLane.collision_free_front, &egoLane.avg_lane_speed, &egoLane.acc_request);
          //std::cout << "Main avg speed: " << egoLane.avg_lane_speed << std::endl;
          //float cost = getEgoLaneCost(max_vel, egoLane.avg_lane_speed);
          //neighborLane1.getFront(sensor_fusion, neighborLane1.lane_width, neighborLane1.ego_s, neighborLane1.ego_speed, &neighborLane1.id_front, &neighborLane1.distance2collision_front, &neighborLane1.target_v_front, &neighborLane1.ttc_front, &neighborLane1.collision_free_front, &neighborLane1.avg_lane_speed);
          //neighborLane1.getRear(sensor_fusion, neighborLane1.lane_width, neighborLane1.ego_s, neighborLane1.ego_speed, &neighborLane1.id_rear, &neighborLane1.distance2collision_rear, &neighborLane1.ttc_rear, &neighborLane1.collision_free_rear);
          //neighborLane2.getFront(sensor_fusion, neighborLane2.lane_width, neighborLane2.ego_s, neighborLane2.ego_speed, &neighborLane2.id_front, &neighborLane2.distance2collision_front, &neighborLane2.target_v_front,  &neighborLane2.ttc_front, &neighborLane2.collision_free_front, &neighborLane2.avg_lane_speed);
          //neighborLane2.getRear(sensor_fusion, neighborLane2.lane_width, neighborLane2.ego_s, neighborLane2.ego_speed, &neighborLane2.id_rear, &neighborLane2.distance2collision_rear, &neighborLane2.ttc_rear, &neighborLane2.collision_free_rear);

          //egoLane.getRear(sensor_fusion, egoLane.lane_width, egoLane.ego_s, egoLane.ego_speed, &egoLane.id_front, &egoLane.distance2collision_front, &egoLane.ttc_front, &egoLane.collision_free_rear);

          //Lane leftLane = Lane(lane - 1, lane_width, car_s, car_speed);
          //leftLane.getFront(sensor_fusion, leftLane.lane_width, leftLane.ego_s, leftLane.ego_speed, &leftLane.id_front, &leftLane.distance2collision_front, &leftLane.ttc_front, &leftLane.collision_free_front);
          //leftLane.getRear(sensor_fusion, leftLane.lane_width, leftLane.ego_s, leftLane.ego_speed, &leftLane.id_front, &leftLane.distance2collision_front, &leftLane.ttc_front, &leftLane.collision_free_rear);

          //Lane rightLane = Lane(lane + 1, lane_width, car_s, car_speed);
          //rightLane.getFront(sensor_fusion, rightLane.lane_width, rightLane.ego_s, rightLane.ego_speed, &rightLane.id_front, &rightLane.distance2collision_front, &rightLane.ttc_front, &rightLane.collision_free_front);
          //rightLane.getRear(sensor_fusion, rightLane.lane_width, rightLane.ego_s, rightLane.ego_speed, &rightLane.id_front, &rightLane.distance2collision_front, &rightLane.ttc_front, &rightLane.collision_free_rear);
          //std::cout << "acc_request: " << egoLane.acc_request << std::endl;
          /*std::cout << "distance2collison: " << egoLane.distance2collision_front << std::endl;
          if (!egoLane.collision_free_front) {
        	  if(abs(egoLane.acc_request) > max_acc){
        		  std::cout << "decc too large! " << std::endl;
            	  set_vel += -0.3;//-max_acc;
        	  }
        	  else{
        		  std::cout << "Use requested decc" << std::endl;
        		  set_vel += -0.3; //egoLane.acc_request;
        	  }
*/

        	  //Evaluate lanes for a possible lane change
        	  //std::cout << "Speed egolane: " << egoLane.avg_lane_speed << std::endl;
   /*       } else if ((max_vel - set_vel > 0.0) && (set_vel < 49.5)) {
        	  set_vel += 0.3;
        	  std::cout << "Accelerate! << std::endl;
          } //else if ((set_vel >= 49.5) && (set_vel <= 49.8)) {
        	  //set_vel += 0.1;
        	  //std::cout << "Accelerating 0.1!" << std::endl;
         // }


          /* ttc = ttc_calculation(sensor_fusion, lane, lane_width, end_path_s, car_s, car_speed, true, false);

          vector<double> ttc_left;
          ttc_left.push_back(999.9);
          ttc_left.push_back(-999.9);
          vector<double> ttc_right;
          ttc_right.push_back(999.9);
          ttc_right.push_back(-999.9);

          // If TTC is too small, check for a possible lane change - if a lane change is not possible, slow down
          if ((ttc[0] <= 3.5) && (ttc[0] > 0.0)){
        	  set_vel += -0.3;
        	  //std::cout << "Decrease the velocity: " << std::endl;
              //std::cout << "TTC: " << ttc[0] << std::endl;
              // check the ttc_s at adjacent lanes
             switch(lane){
                   case 0: ttc_left[0] = -999.9;
                           ttc_left[1] = 999.9;
                           ttc_right = ttc_calculation(sensor_fusion, 1, lane_width, end_path_s, car_s, car_speed, true, false);
                           lane_free_right = lane_occupancy(sensor_fusion, 1, car_s, lane_width);
                           if ((lane_free_right == true)){ //&& (ttc_right[0] > ttc[0]) && (ttc_right[1] < -1.2)){
                        	   target_lane_idx = 1; // change lane to the left
                           }
                           else{
                        	   target_lane_idx = 0; // stay in the current lane
                           }
                           break;
                   case 1: ttc_left = ttc_calculation(sensor_fusion, 0, lane_width, end_path_s, car_s, car_speed, true, true);
                           ttc_right = ttc_calculation(sensor_fusion, 2, lane_width, end_path_s, car_s, car_speed, true, true);
                           lane_free_left = lane_occupancy(sensor_fusion, 0, car_s, lane_width); // check the occupancy of the left lane
                           lane_free_right = lane_occupancy(sensor_fusion, 2, car_s, lane_width); // check the occupancy of the right lane
                           if ((lane_free_left == true)){// && (ttc_left[0] > ttc[0]) && (ttc_left[1] < -1.2)){ // if left lane is free and the ttc to the front and rear vehicles is large enough
                        	   target_lane_idx = 0; // change lane to the left
                           }
                           else if ((lane_free_right == true)){ //&& (ttc_right[0] > ttc[0]) && (ttc_right[1] < -1.2)){ // if right lane is free and the ttc to the front and rear vehicles is large enough
                        	   target_lane_idx = 2; // change lane to the right
                           }
                           else{
                        	   target_lane_idx = 1; // stay in the current lane
                           }
                           //std::cout << "Left lane free: " << lane_free_left << std::endl;
                           //std::cout << "Right lane free: " << lane_free_right << std::endl;
                           break;
                   case 2: ttc_left = ttc_calculation(sensor_fusion, 1, lane_width, end_path_s, car_s, car_speed, true, true);
                           ttc_right[0] = -999.9;
                           ttc_right[1] = 999.9;
                           lane_free_left = lane_occupancy(sensor_fusion, 1, car_s, lane_width);
                           if ((lane_free_left == true)){// && (ttc_left[0] > ttc[0]) && (ttc_left[1] < -2.0)){ // if left lane is free and the ttc to the front and rear vehicles is large enough
                        	   target_lane_idx = 1; // change lane to the left
                           }
                           else{
                        	   target_lane_idx = 2; // stay in the current lane
                           }
                           break;
             }
             lane = target_lane_idx;
          }
          else{ // if the ttc is high enough, accelerate to achieve the max speed
        	  if ((max_vel - set_vel > 0.0) && (set_vel < 49.5)) {
        		  set_vel += 0.3;
        	  }
          }
          */


          /** DISCLAIMER: This part of the code was implemented based on the ideas of Udacity instructors from the Q&A video session (https://www.youtube.com/watch?v=7sI3VHFPP0w&feature=youtu.be)
           *
           */
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
