#include "planner.hpp"
#include <vector>
#include <string>
#include <iostream>

using std::vector;

// get the front target vehicle
void Lane::getFront(vector<vector<double>> sensor_fusion, int lane_width, double ego_s, double ego_v, int *id, double *distance2collision, double *ttc_front, bool *collision_free_front){
	*id = 99;
	*distance2collision = 9999.9;
	*ttc_front = 99.9;
	for (int i = 0; i < sensor_fusion.size(); i++) {
			// Check for all the objects within the ego lane
			if ((sensor_fusion[i][6] >= lane_width * this->lane_id)
					&& (sensor_fusion[i][6] <= (this->lane_id + 1) * lane_width)) {
				double target_vx = sensor_fusion[i][3];
				double target_vy = sensor_fusion[i][4];
				double target_s = sensor_fusion[i][5];
				double target_speed = sqrt(target_vx * target_vx + target_vy * target_vy) / 2.24;
				double dist2ego = target_s - ego_s;

				if (dist2ego >= 0.0) { // the target car is in front of the ego

					if (*distance2collision > dist2ego) {
						*id = i;
						*distance2collision = dist2ego;
						*ttc_front  = dist2ego / (ego_v / 2.24 - target_speed);
					}
				}else{
					*id = 99;
				}
			}
	}
	if (*ttc_front > 3.0){
		*collision_free_front = true;
	} else {
		*collision_free_front = false;
	}
}
void Lane::getRear(vector<vector<double>> sensor_fusion, int lane_width, double ego_s, double ego_v, int *id, double *distance2collision, double *ttc_rear, bool *collision_free_rear){
	*id = 99;
	*distance2collision = -9999.9;
	*ttc_rear = -99.9;
	for (int i = 0; i < sensor_fusion.size(); i++) {
			// Check for all the objects within the ego lane
			if ((sensor_fusion[i][6] >= lane_width * this->lane_id)
					&& (sensor_fusion[i][6] <= (this->lane_id + 1) * lane_width)) {
				double target_vx = sensor_fusion[i][3];
				double target_vy = sensor_fusion[i][4];
				double target_s = sensor_fusion[i][5];
				double target_speed = sqrt(target_vx * target_vx + target_vy * target_vy) / 2.24;
				double dist2ego = target_s - ego_s;

				if (dist2ego < 0.0) { // the target car is behind ego

					if (*distance2collision < dist2ego) {
						*id = i;
						*distance2collision = dist2ego;
						*ttc_rear  = dist2ego / (ego_v / 2.24 - target_speed);
						std::cout << "Target (rear) found, TTC: " << *ttc_rear << std::endl;
					}
				}else{
					*id = 99;
				}
			}
	}
	if (*ttc_rear < -1.0){
		*collision_free_rear = true;
	} else {
		*collision_free_rear = false;
	}
}
/*
double Lane::speed_efficiency(vector<vector<double>> sensor_fusion, int lane_id, int lane_width, double ego_s){

	double distance2collision_rear;
	for (int i = 0; i < sensor_fusion.size(); i++) {
		// Check for all the objects within the ego lane
		if ((sensor_fusion[i][6] >= lane_width * lane_id)
				&& (sensor_fusion[i][6] <= (lane_id + 1) * lane_width)) {
			double target_vx = sensor_fusion[i][3];
			double target_vy = sensor_fusion[i][4];
			double target_s = sensor_fusion[i][5];
			double dist2ego = target_s - ego_s;

			if (dist2ego >= 0.0) { // the target car is in front of the ego

				if (distance2collision_front > dist2ego) {
					target_idx_front = i;
					distance2collision_front = dist2ego;
				}
			} else { // the target car is behind the ego
				if (distance2collision_rear < dist2ego) {
					distance2collision_rear = dist2ego;
				}
			}
		}
}
*/
