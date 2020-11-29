#include "planner.hpp"
#include <vector>
#include <string>
#include <iostream>
#include <numeric>

using namespace std;
using std::vector;

vector<Lane::object> Lane::Obj2LaneAssignment(vector<vector<double>>sensor_fusion, double lane_id, double lane_width, double ego_s,
		double ego_v, int *target_id, double *delta_v_target, double *target_dist, double *target_d) {
	vector<object> lane_objects;
	object fusion_objects;
	double distance2collision_front = 99999.9;
	double distance2collision_rear = -99999.9;
	double delta_v = 99.9;
	double dist2ego = 99999.9;
	*target_id = 99;
	*target_dist = 99999.99;
	*delta_v_target = 99999.99;
	*target_d = 99.9;
	for (int i = 0; i < sensor_fusion.size(); i++){
		if ((sensor_fusion[i][6] >= (lane_width * lane_id - 0.1))
				&& (sensor_fusion[i][6] <= ((lane_id + 1) * lane_width + 0.1))){
			fusion_objects.id = sensor_fusion[i][0];
			fusion_objects.pos_x = sensor_fusion[i][1];
			fusion_objects.pos_y = sensor_fusion[i][2];
			fusion_objects.v_x = sensor_fusion[i][3];
			fusion_objects.v_y = sensor_fusion[i][4];
			fusion_objects.pos_s = sensor_fusion[i][5];
			fusion_objects.pos_d = sensor_fusion[i][6];


			lane_objects.push_back(fusion_objects);
			// now find the target object and its delta speed
			double target_vx = sensor_fusion[i][3];
			double target_vy = sensor_fusion[i][4];
			double target_s = sensor_fusion[i][5];
			double target_speed_m_s = sqrt(
					target_vx * target_vx + target_vy * target_vy); //
			double target_speed_mph = target_speed_m_s * 2.24;
			dist2ego = target_s - ego_s - 3.0; // vehicle length is approx. 4.5

			// Define the target object in this lane

			if (dist2ego >= 0.0) {

				if (distance2collision_front > dist2ego) {
					distance2collision_front = dist2ego;
					delta_v = ego_v - target_speed_mph; // miles per hour
					int id_target = sensor_fusion[i][0];
					*target_id = id_target;
					*target_dist = distance2collision_front;
					*delta_v_target = delta_v;
					*target_d = fusion_objects.pos_d;
				}
			}
			else{
			}
		}
	}
	return lane_objects;
}

double Lane::rear_traffic_check(vector<vector<double>>sensor_fusion, double lane_id, double lane_width, double ego_s,
		double ego_v){
	vector<object> lane_objects;
	object fusion_objects;
	double distance2collision_rear = -99999.9;
	double delta_v = 99.9;
	double dist2ego = -99999.9;
	for (int i = 0; i < sensor_fusion.size(); i++){
		if ((sensor_fusion[i][6] >= (lane_width * lane_id - 0.1))
				&& (sensor_fusion[i][6] <= ((lane_id + 1) * lane_width + 0.1))){
			fusion_objects.id = sensor_fusion[i][0];
			fusion_objects.pos_x = sensor_fusion[i][1];
			fusion_objects.pos_y = sensor_fusion[i][2];
			fusion_objects.v_x = sensor_fusion[i][3];
			fusion_objects.v_y = sensor_fusion[i][4];
			fusion_objects.pos_s = sensor_fusion[i][5];
			fusion_objects.pos_d = sensor_fusion[i][6];
			// now find the target object and its delta speed
			double target_rear_s = sensor_fusion[i][5];
			dist2ego = target_rear_s - ego_s - 3.0; // vehicle length is approx. 4.5
			// Define the target object in this lane
			if (dist2ego < 0.0) {
				if (distance2collision_rear < dist2ego) {
					distance2collision_rear = dist2ego;
					int id_target = sensor_fusion[i][0];
				}
			}
			else{
			}
		}
	}
	return distance2collision_rear;
}
