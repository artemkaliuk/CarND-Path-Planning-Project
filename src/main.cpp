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

bool lane_occupancy(vector<vector<double>> sensor_fusion, int lane, double car_s, int lane_width){
	bool lane_free_front = false;
	bool lane_free_rear = false;
	double distance2collision_front = 9999.9;
	double distance2collision_rear = -9999.9;
	double dist2ego = 9999.9;
	int target_idx_front;

	for (int i = 0; i < sensor_fusion.size(); i++){ // go through the object list, find objects within the target lane and find longitudinal distance between the ego car and the object
		if ((sensor_fusion[i][6] >= lane_width * lane) && (sensor_fusion[i][6] <= (lane + 1) * lane_width)){
			double target_vx = sensor_fusion[i][3];
			double target_vy = sensor_fusion[i][4];
			double target_s = sensor_fusion[i][5];
			dist2ego = target_s - car_s;

			if (dist2ego >= 0.0) { // the target car is in front of the ego

				if (distance2collision_front > dist2ego) {
					target_idx_front = i;
					distance2collision_front = dist2ego;
				}
			}
			else{ // the target car is behind the ego
				if (distance2collision_rear < dist2ego){
					distance2collision_rear = dist2ego;
				}
			}
		}
	}

	if (distance2collision_front > 30.0) {
		lane_free_front = true;
	} else {
		lane_free_front = false;
	}

	if (distance2collision_rear < -10) {
		lane_free_rear = true;
	} else {
		lane_free_rear = false;
	}
	return lane_free_front && lane_free_rear;
}

vector<double> ttc_calculation(vector<vector<double>> sensor_fusion, int lane, int lane_width, double end_path_s, double car_s, double car_speed, bool front, bool rear){
    // Avoid collisions
    // 1. Iterate through all the cars and check if they are in our lane
    // 2. For the cars in our lane: check for the time gap - if the time gap is smaller than expected, adapt the target speed
    // 3. Feed the target speed in to the waypoint generator

    int idx = 99;
    int idx_rear = 99;
    double distance2collision = 9999.9;
    vector <double> ttc;
    ttc.push_back(99.9); // initializing TTC for the front area
    ttc.push_back(-99.9); // initializing TTC for the rear area
    for (int i = 0; i < sensor_fusion.size(); i++){
  	  // Check for all the objects within the ego lane
  	  if ((sensor_fusion[i][6] >= lane_width * lane) && (sensor_fusion[i][6] <= (lane + 1) * lane_width)){
  		  double target_vx = sensor_fusion[i][3];
  		  double target_vy = sensor_fusion[i][4];
  		  double target_s = sensor_fusion[i][5];
  		  double delta_s = target_s - car_s;
  		  // Choose the object in the target lane that is closest to the ego car
  		  if (distance2collision > abs(delta_s)){
  			  // Check front targets
				if (front == true) {
					if (delta_s > 0) {
						idx = i;
						//std::cout << "Target found" << std::endl;
						distance2collision = delta_s;
					} else {
						idx = 99;
					}
				}
				// Check the rear targets
				if (rear == true) {
					if (delta_s < 0) {
						idx_rear = i;
					} else {
						idx_rear = 99;
					}
				}
  		  }
  	  }
    }

    if (idx != 99){
     	  // Calculate the time to collision metric
  	  double target_vx = sensor_fusion[idx][3];
  	  double target_vy = sensor_fusion[idx][4];
      double target_s = sensor_fusion[idx][5];
  	  double target_speed = sqrt(target_vx * target_vx + target_vy * target_vy)/2.24;
  	  // Calculate the distance between the target vehicle and the last point of the planned path
  	  double s_diff = target_s - car_s; // calculate the distance between the ego vehicle and the target
  	  double speed_diff = car_speed / 2.24 - target_speed;// calculate the speed difference between the ego and the target

  	  if (abs(speed_diff) < .001){ // division by zero check
  		  speed_diff = .01;
  	  }

  	  //std::cout << "Distance2Target" << s_diff << std::endl;
  	  // if the target car is far away and the ego car is too slow, set the time to collision to a large value
  	  if ((s_diff > 80) || (speed_diff < -5.0)){
  		  ttc[0] = 999.9;
  	  }
  	  else{
  		  ttc[0] = s_diff / (speed_diff);
  	  }
    }
    if (idx_rear != 99){
         	  // Calculate the time to collision metric
      	  double target_vx = sensor_fusion[idx_rear][3];
      	  double target_vy = sensor_fusion[idx_rear][4];
          double target_s = sensor_fusion[idx_rear][5];
      	  double target_speed = sqrt(target_vx * target_vx + target_vy * target_vy)/2.24;
      	  // Calculate the distance between the target vehicle and the last point of the planned path
      	  double s_diff = target_s - car_s; // calculate the distance between the ego vehicle and the target
      	  double speed_diff = car_speed / 2.24 - target_speed; // calculate the speed difference between the ego vehicle and the target

      	  if (abs(speed_diff) < .001){ // division by zero check
      		  speed_diff = .01;
      	  }

      	  std::cout << "Distance2Target" << s_diff << std::endl;
      	  // if the target car is far away and the ego car is too slow, set the time to collision to a large value
      	  if ((s_diff < -30) || ((car_speed / 2.24 - target_speed) > 5.0)){
      		  ttc[1] = 999.9;
      	  }
      	  else{
      		  ttc[1] = s_diff / (car_speed / 2.24 - target_speed);
      	  }
        }

    //std::cout<<"TTC: " << ttc[0] << std::endl;
    return ttc;
}

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
  static int lane = 1;
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

          /**2
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

          int lane_width = 4;
          int target_lane_idx;
          double anchor_spacing = 50.0; // spacing between the anchor points of the spline for smoother trajectories
          double dist_inc = 0.3; // spacing between the waypoints

          Lane egoLane = Lane(lane, lane_width, car_s, car_speed);
 //         egoLane.lane_id = lane;
 //         egoLane.lane_width = lane_width;
 //         egoLane
          egoLane.getFront(sensor_fusion, egoLane.lane_width, egoLane.ego_s, egoLane.ego_speed, &egoLane.id_front, &egoLane.distance2collision_front, &egoLane.ttc_front, &egoLane.collision_free_front);
          egoLane.getRear(sensor_fusion, egoLane.lane_width, egoLane.ego_s, egoLane.ego_speed, &egoLane.id_front, &egoLane.distance2collision_front, &egoLane.ttc_front, &egoLane.collision_free_rear);

          if (!egoLane.collision_free_front) {
        	  set_vel += -0.3;

        	  std::cout << "Target (front) found, TTC: " << egoLane.ttc_front << std::endl;
          } else if ((max_vel - set_vel > 0.0) && (set_vel < 48.5)) {
        	  set_vel += 1.0;
        	  std::cout << "Accelerating 1.0!" << std::endl;
          } else if ((set_vel >= 49.5) && (set_vel <= 49.9)) {
        	  set_vel += 0.1;
        	  std::cout << "Accelerating 0.1!" << std::endl;
          }


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
