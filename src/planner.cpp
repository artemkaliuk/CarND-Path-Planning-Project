#include "planner.hpp"
#include <vector>
#include <string>
#include <iostream>
#include <numeric>

using namespace std;
using std::vector;

// get the front target vehicle
void Lane::getFront(vector<vector<double>> sensor_fusion, int lane_width, double ego_s, double ego_v, int *id, double *distance2collision, double *ttc_front, bool *collision_free_front){
	*id = 99;
	*distance2collision = 9999.9;
	*ttc_front = 99.9;
	double lane_speed = 0;
	int idx = 0;
	if ((this->lane_id < 0) || (this->lane_id > 2)){
		*collision_free_front = false;
		std::cout << this->lane_id << std::endl;
	}else
	{
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

				if (dist2ego >= 0.0) { // the target car is in front of the ego
					lane_speed += target_speed;
					idx += 1;

					if (*distance2collision > dist2ego) {
						*id = i;
						*distance2collision = dist2ego;
						double delta_v = ego_v / 2.24 - target_speed;
						if (delta_v < 0.001){
							*ttc_front = 99.9;
						}
						else{
							*ttc_front = dist2ego / delta_v;
						}
					}
				} else {
					*id = 99;
				}
			}
		}
		if (idx != 0){ // Division by zero check
			this->avg_lane_speed = lane_speed / idx; // calculate the average lane speed for this lane
		}

		if (*ttc_front > 3.0) {
			*collision_free_front = true;
		} else {
			*collision_free_front = false;
			std::cout << "ttc_front: " << *ttc_front << std::endl;
		}
	}
}

void Lane::getRear(vector<vector<double>> sensor_fusion, int lane_width,
		double ego_s, double ego_v, int *id, double *distance2collision,
		double *ttc_rear, bool *collision_free_rear) {
	*id = 99;
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
						std::cout << "Target (rear) found, TTC: " << *ttc_rear
								<< std::endl;
					}
				} else {
					*id = 99;
				}
			}
		}
		if (*ttc_rear < -1.0) {
			*collision_free_rear = true;
		} else {
			*collision_free_rear = false;
		}
	}
}

/*
vector<double> Lane::lane_speed(vector<vector<double>> sensor_fusion,
		int lane_width, double ego_s) {

	double distance2collision_rear;
	vector<vector<double>> object_speed;
	vector<double> avg_speed;
	for (int j = 0; j < 3; j++) { // iterate over three lanes
		for (int i = 0; i < sensor_fusion.size(); i++) {
			// Check for all the objects within the ego lane
			if ((sensor_fusion[i][6] >= lane_width * lane_id)
					&& (sensor_fusion[i][6] <= (lane_id + 1) * lane_width)) {
				double target_vx = sensor_fusion[i][3];
				double target_vy = sensor_fusion[i][4];
				double target_s = sensor_fusion[i][5];
				double dist2ego = target_s - ego_s;

				if (dist2ego >= 0.0) { // the target car is in front of the ego
					double target_speed = sqrt(
							target_vx * target_vx + target_vy * target_vy)
							/ 2.24;
					object_speed[j].push_back(target_speed);
				}
			}
			// Find average lane speed for this lane
			if (object_speed[j].size() != 0) {
				double mean = accumulate(object_speed[j].begin(),
						object_speed[j].end(), 0) / object_speed[j].size();
				avg_speed.push_back(mean);
			}

		}
	}
	return avg_speed;
}
*/
