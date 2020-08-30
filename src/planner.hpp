#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <string>
#include <vector>

using std::vector;

struct lane_identifiers
{
	enum lane_enum {left = 0, middle = 1, right = 2};
};

class Lane{
public:
	// Lane functions
//

	//int target_in_lane(vector<vector<double>> sensor_fusion, int lane_id, int lane_width, double ego_s);
	//double speed_efficiency(vector<vector<double>> sensor_fusion, int lane_id, int lane_width, double ego_s);
	int target_id;
	double ego_s, ego_d, ego_v, target_s, target_d, target_delta_v, lane_id, lane_width;

	struct object{
		int id;
		double pos_x, pos_y, v_x, v_y, pos_s, pos_d;
	};

	// find the ids of the objects located on this particular lane
	vector<object> Obj2LaneAssignment(vector<vector<double>>sensor_fusion, double lane_id, double lane_width, double ego_s, double ego_v, int *target_id, double *delta_v_target, double *target_s, double *target_d);

	Lane(double id, double width, double car_s, double car_d, double car_speed, double max_allowed_speed, vector<vector<double>>sensor_fusion){
			lane_id = id;
			lane_width = width;
			ego_s = car_s;
			ego_d = car_d;
			ego_v = car_speed;
			vector<int> obj_id;
			double delta_v_target;
			// Find out the

			vector<object> lane_objects;
			lane_objects = Obj2LaneAssignment(sensor_fusion, lane_id, lane_width, ego_s, ego_v, &target_id, &target_delta_v, &target_s, &target_d);
		}
};

//float getEgoLaneCost(double max_speed, double avg_speed);
