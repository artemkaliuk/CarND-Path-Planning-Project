#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <string>
#include <vector>

using std::vector;

class Lane{
public:
	// Lane functions
//

	//int target_in_lane(vector<vector<double>> sensor_fusion, int lane_id, int lane_width, double ego_s);
	//double speed_efficiency(vector<vector<double>> sensor_fusion, int lane_id, int lane_width, double ego_s);
	int lane_id, lane_width, id_front, id_rear;
	double ego_s, ego_speed, distance2collision_front, distance2collision_rear, ttc_front, ttc_rear, avg_lane_speed;
	bool collision_free_front, collision_free_rear;
	Lane(int id, int width, double car_s, double car_speed){
			lane_id = id;
			lane_width = width;
			ego_s = car_s;
			ego_speed = car_speed;
			id_front = 99;
			id_rear = 99;
			distance2collision_front = 999.9;
			distance2collision_rear = -999.9;
			ttc_front = 999.9;
			ttc_rear = -999.9;
			collision_free_front = false;
			collision_free_rear = false;
			avg_lane_speed = 50.0;
		}
	void getFront(vector<vector<double>> objects, int lane_width, double ego_s, double ego_v, int *id_front, double *distance2collision_front, double *ttc_front, bool *collision_free_front);
    void getRear(vector<vector<double>> objects, int lane_width, double ego_s, double ego_v, int *id_rear, double *distance2collision_rear, double *ttc_rear, bool *collision_free_rear);

};
