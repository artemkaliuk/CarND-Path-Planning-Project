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
				//std::cout << "Target not available 2" << std::endl;
			}
		}
	}
	//std::cout << "Obj ID: " << *target_id << " Obj d: " << *target_d << " Delta s: " << *target_dist << std::endl;
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
				//double target_rear_vx = sensor_fusion[i][3];
				//double target_rear_vy = sensor_fusion[i][4];
				double target_rear_s = sensor_fusion[i][5];
				//double target_speed_m_s = sqrt(
						//target_vx * target_vx + target_vy * target_vy); //
				//double target_speed_mph = target_speed_m_s * 2.24;
				dist2ego = target_rear_s - ego_s - 3.0; // vehicle length is approx. 4.5
				//double target_speed_m_s = sqrt(target_vx * target_vx + target_vy * target_vy); //
				//double target_speed_mph = target_speed_m_s * 2.24;
				// Define the target object in this lane
				if (dist2ego < 0.0) {
					if (distance2collision_rear < dist2ego) {
						distance2collision_rear = dist2ego;
						//delta_v = ego_v - target_speed_mph; // miles per hour
						int id_target = sensor_fusion[i][0];
						//*target_id = id_target;
						//*target_dist = distance2collision_front;
						//*delta_v_target = delta_v;
						//*target_d = fusion_objects.pos_d;
					}
				}
				else{
								//std::cout << "Target not available 2" << std::endl;
					}
		}
	}
					//std::cout << "Obj ID: " << *target_id << " Obj d: " << *target_d << " Delta s: " << *target_dist << std::endl;
	return distance2collision_rear;
}


/*	double safe_dist = -9999.9;
	double ttc = 2.0;
	double lane_speed = 0;
	int idx = 0;
	double *distance2collision = 99999.9;
	if ((this->lane_id < 0) || (this->lane_id > 2)) {
//		*collision_free_front = false;
//		*avg_lane_speed = 0.0;
	} else {
		for (int i = 0; i < sensor_fusion.size(); i++) {
			// Check for all the objects within the ego lane
			if ((sensor_fusion[i][6] >= lane_width * this->lane_id)
					&& (sensor_fusion[i][6] <= (this->lane_id + 1) * lane_width)) {
				double target_vx = sensor_fusion[i][3];
				double target_vy = sensor_fusion[i][4];
				double target_s = sensor_fusion[i][5];
				double target_speed = sqrt(
						target_vx * target_vx + target_vy * target_vy); //
				double target_speed_mph = target_speed * 2.24;
				double dist2ego = target_s - ego_s - 4.5;

				// Define the target object in this lane
				if (dist2ego >= 0.0) {
					if (*distance2collision > dist2ego) {
						*distance2collision = dist2ego;
						*target_v = target_speed_mph;
						double delta_v = ego_v / 2.24 - target_speed; // meters per second
						safe_dist = 30.0;
						std::cout << "Dist2coll: " << *distance2collision << "Safe distance: " << safe_dist << std::endl;
						if (*distance2collision < safe_dist) {
							*collision_free_front = false;
							*acc_request = (*distance2collision - safe_dist)
									/ (1609.344); // miles per frame
						} else {
							*collision_free_front = true;
							*acc_request = 0.0;
						}
					}
				}
			}
		}
	}
	*/


/*void Lane::getRear(vector<vector<double>> sensor_fusion, int lane_width,
		double ego_s, double ego_v, int *id, double *distance2collision,
		double *ttc_rear, bool *collision_free_rear) {

	*distance2collision = -9999.9;
	*ttc_rear = -99.9;

	if ((this->lane_id < 0) || (this->lane_id > 2)) {
		*collision_free_rear = false;
	} else {
		for (int i = 0; i < sensor_fusion.size(); i++) {
			// Check for all the objects within the ego lane
			if ((sensor_fusion[i][6] >= lane_width * this->lane_id)
					&& (sensor_fusion[i][6] <= (this->lane_id + 1) * lane_width)) {
				double target_vx = sensor_fusion[i][3];
				double target_vy = sensor_fusion[i][4];
				double target_s = sensor_fusion[i][5];
				double target_speed = sqrt(
						target_vx * target_vx + target_vy * target_vy) / 2.24;
				double dist2ego = target_s - ego_s;

				if (dist2ego < 0.0) { // the target car is behind ego

					if (*distance2collision < dist2ego) {
						*id = i;
						*distance2collision = dist2ego;
						double delta_v = ego_v / 2.24 - target_speed;
						if (delta_v < 0.001) {
							*ttc_rear = 99.9;
						} else {
							*ttc_rear = dist2ego / delta_v;
						}
					}
				} else {
					*id = 99;
				}
			}
		}
		if ((*ttc_rear < -1.0) && (*distance2collision < -15.0)) {
			*collision_free_rear = true;
		} else {
			*collision_free_rear = false;
		}
	}
}

float getEgoLaneCost(double max_speed, double avg_speed_lane){

	float cost = 1 - exp(-(max_speed - avg_speed_lane));

	return cost;
}
*/
