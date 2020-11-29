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
	double ego_s, ego_d, ego_v, target_s, target_d, target_delta_v, lane_id, lane_width, rear_traffic_dist;

	struct object{
		int id;
		double pos_x, pos_y, v_x, v_y, pos_s, pos_d;
	};

	// find the ids of the objects located on this particular lane
	vector<object> Obj2LaneAssignment(vector<vector<double>>sensor_fusion, double lane_id, double lane_width, double ego_s, double ego_v, int *target_id, double *delta_v_target, double *target_s, double *target_d);
	double rear_traffic_check(vector<vector<double>>sensor_fusion, double lane_id, double lane_width, double ego_s,
			double ego_v);

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
			rear_traffic_dist = rear_traffic_check(sensor_fusion, lane_id, lane_width, ego_s, ego_v);
	}

};



// Here a circular buffer class is defined in order to keep and process the measurements.
// Implementation is taken from
// https://embeddedartistry.com/blog/2017/05/17/creating-a-circular-buffer-in-c-and-c/

/*
template <class T>
class circular_buffer {
public:
	explicit circular_buffer(size_t size) :
		buf_(std::unique_ptr<T[]>(new T[size])),
		max_size_(size)
	{ // empty }

	void put(T item);
	T get();
	void reset();
	bool empty() const;
	bool full() const;
	size_t capacity() const;
	size_t size() const;

private:
	std::mutex mutex_;
	std::unique_ptr<T[]> buf_;
	size_t head_ = 0;
	size_t tail_ = 0;
	const size_t max_size_;
	bool full_ = 0;
};

explicit circular_buffer(size_t size) :
		buf_(std::unique_ptr<T[]>(new T[size])),
		max_size_(size)
{
		//empty constructor
	}
	*/
//float getEgoLaneCost(double max_speed, double avg_speed);
