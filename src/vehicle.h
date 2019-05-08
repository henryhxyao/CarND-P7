#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <fstream>
using std::vector;
using std::string;

struct Trajectory {
  // action_name
  string action;
  
  // s,d trajectory
  vector<double> next_s_vals;
  vector<double> next_d_vals;

  // x,y trajectory output to the simulator
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  double target_lane;
  double dist_lane_change;
};

struct PredictedTrajectory {
  int id;
  vector<double> predicted_s;
  vector<double> predicted_d;
};

class Vehicle {
public:

Vehicle(const vector<double> &input_map_waypoints_x,
		const vector<double> &input_map_waypoints_y,
		const vector<double> &input_map_waypoints_s,
		const vector<double> &input_map_waypoints_dx,
		const vector<double> &input_map_waypoints_dy);

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

void generatePrediction(const vector<vector<double>> &sensor_fusion);
void FSMPlanner();
vector<string> successorStates();  // states of the Finite States Machine
Trajectory splineTrajectoryGen(const double goal_vel, const string action, const vector<double> &anchor_waypoint_lane, const vector<double> &anchor_waypoint_s);
//Trajectory quinticPolynomialTrajectoryGen(double goal_vel, double goal_lane);

// behavior cost functions
double calculateCost(const Trajectory &candidate_trajectory);
double calculateCostObstacleAvoidance(const Trajectory &candidate_trajectory);
double calculateCostTrafficJam(const Trajectory &candidate_trajectory);
//double calculateCostComfort(const Trajectory &candidate_trajectory);

// waypoints info
vector<double> map_waypoints_x;
vector<double> map_waypoints_y;
vector<double> map_waypoints_s;
vector<double> map_waypoints_dx;
vector<double> map_waypoints_dy;

// vehicle current state
double car_x;
double car_y;
double car_s;
double car_d;
double car_yaw;
double car_speed;

// current step prediction of surrounding vehicles
double horizon = 3;
vector<PredictedTrajectory> prediction;
vector<double> distances_ahead;

// previous path
vector<double> previous_path_x;
vector<double> previous_path_y;
int prev_size;
double end_path_s;
double end_path_d;

// best trajectory
Trajectory best_trajectory;
double ref_vel = 0.2;
double max_vel = 22.0; // 49.2mph

};

#endif  //VEHICLE_H