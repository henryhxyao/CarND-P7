#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <fstream>
using std::vector;
using std::string;

struct Trajectory {
  // s,d trajectory
  vector<double> next_s_vals;
  vector<double> next_d_vals;

  // x,y trajectory output to the simulator
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  double ref_vel;
  double dist_lane_change;
};

struct PredictedTrajectory {
  int id;
  vector<double> predicted_x;
  vector<double> predicted_y;
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
Trajectory splineTrajectoryGen(double goal_vel, double goal_lane);
Trajectory quinticPolynomialTrajectoryGen(double goal_vel, double goal_lane);

// behavior cost functions
double calculateCost(const Trajectory &candidate_trajectory);
double calculateCostSpeed(const Trajectory &candidate_trajectory);
double calculateCostChangeLane(const Trajectory &candidate_trajectory);
double calculateCostObstacleAvoidance(const Trajectory &candidate_trajectory);
double calculateCostTrafficJam(const Trajectory &candidate_trajectory);

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

// previous path
vector<double> previous_path_x;
vector<double> previous_path_y;
int prev_size;
double end_path_s;
double end_path_d;

// best trajectory
Trajectory best_trajectory;

std::ofstream myfile_xy;
std::ofstream myfile_sd;

};

#endif  //VEHICLE_H