#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
using std::vector;

class Vehicle {
public:

Vehicle(const vector<double> &input_map_waypoints_x,
		const vector<double> &input_map_waypoints_y,
		const vector<double> &input_map_waypoints_s,
		const vector<double> &input_map_waypoints_dx,
		const vector<double> &input_map_waypoints_dy,
		const double &input_ref_vel,
	    const int &input_lane);

virtual ~Vehicle(){};

void setState(const double &input_car_x,
			  const double &input_car_y,
			  const double &input_car_s,
			  const double &input_car_d,
			  const double &input_car_yaw,
			  const double &input_car_speed,
			  const vector<double> &input_previous_path_x,
			  const vector<double> &input_previous_path_y,
			  const double &input_end_path_s,
			  const double &input_end_path_d);

void FSMPlanner(const vector<vector<double>> &sensor_fusion);
bool splineTrajectoryGen(vector<double> &next_x_vals, vector<double> &next_y_vals);

// control value
double ref_vel;
double lane;

// vehicle current state
double car_x;
double car_y;
double car_s;
double car_d;
double car_yaw;
double car_speed;

// previous path
vector<double> previous_path_x;
vector<double> previous_path_y;
int prev_size;
double end_path_s;
double end_path_d;

// waypoints info
vector<double> map_waypoints_x;
vector<double> map_waypoints_y;
vector<double> map_waypoints_s;
vector<double> map_waypoints_dx;
vector<double> map_waypoints_dy;
};

#endif  //VEHICLE_H