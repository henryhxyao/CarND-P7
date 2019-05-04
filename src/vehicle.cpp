#include <vector>
#include <string>
#include <math.h> 
#include <iostream>
#include <algorithm>
#include "vehicle.h"
#include "helpers.h"
#include "spline.h"
using std::vector;
using std::string;
using std::cout;
using std::endl;

Vehicle::Vehicle(const vector<double> &input_map_waypoints_x,
				const vector<double> &input_map_waypoints_y,
				const vector<double> &input_map_waypoints_s,
				const vector<double> &input_map_waypoints_dx,
				const vector<double> &input_map_waypoints_dy) {

map_waypoints_x = std::move(input_map_waypoints_x);
map_waypoints_y = std::move(input_map_waypoints_y);
map_waypoints_s = std::move(input_map_waypoints_s);
map_waypoints_dx = std::move(input_map_waypoints_dx);
map_waypoints_dy = std::move(input_map_waypoints_dy);
best_trajectory.ref_vel = 0.0;
}

void Vehicle::setState(const double &input_car_x,
					  const double &input_car_y,
					  const double &input_car_s,
					  const double &input_car_d,
					  const double &input_car_yaw,
					  const double &input_car_speed,
					  const vector<double> &input_previous_path_x,
					  const vector<double> &input_previous_path_y,
					  const double &input_end_path_s,
					  const double &input_end_path_d){


car_x = input_car_x;
car_y = input_car_y;
car_s = input_car_s;
car_d = input_car_d;
car_yaw = input_car_yaw;
car_speed = input_car_speed;
previous_path_x = std::move(input_previous_path_x);
previous_path_y = std::move(input_previous_path_y);
prev_size = input_previous_path_x.size();
end_path_s = input_end_path_s;
end_path_d = input_end_path_d;
}

void Vehicle::generatePrediction(const vector<vector<double>> &sensor_fusion) {
  prediction.clear();
  for (int i = 0; i< sensor_fusion.size(); ++i) {
    double detected_car_x = sensor_fusion[i][1];
    double detected_car_y = sensor_fusion[i][2];
    if (distance(car_x, car_y, detected_car_x, detected_car_y)<100.0) {
      PredictedTrajectory check_prediction;
      check_prediction.id = sensor_fusion[i][0];
  	  double vx = sensor_fusion[i][3];
  	  double vy = sensor_fusion[i][4];
  	  double check_car_s = sensor_fusion[i][5];  
  	  float check_car_d = sensor_fusion[i][6];
  	  double check_speed = sqrt(vx*vx + vy*vy);
      check_prediction.start_s = check_car_s;

      for(int i = 0; i < horizon*50; ++i) {
        double check_car_s_predict = check_car_s + (i * 0.02 * check_speed);
        vector<double> check_car_xy = getXY(check_car_s_predict, check_car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        check_prediction.predicted_x.push_back(check_car_xy[0]);
        check_prediction.predicted_y.push_back(check_car_xy[1]);
      }
      check_prediction.final_d = check_car_d;
      check_prediction.final_s = check_car_s + ((horizon * 50-1) * 0.02 * check_speed);
      prediction.push_back(check_prediction);
    }
  }
}

void Vehicle::FSMPlanner() {

  vector<string> candidate_states = successorStates();
  cout << "**********************************************" << endl;
  for(int i=0; i<candidate_states.size(); i++) {
  	cout << candidate_states[i] << endl;
  }
  float cost;
  vector<float> costs;
  vector<Trajectory> final_trajectories;
  double current_lane = floor(car_d/4);

  for(int i = 0; i < candidate_states.size(); ++i) {
    double goal_lane;
    if (candidate_states[i] == "KL") {
    	goal_lane = current_lane;
    }
    else if(candidate_states[i] == "LCL") {
    	goal_lane = current_lane - 1;    	
    }
    else if (candidate_states[i] == "LCR") {
    	goal_lane = current_lane + 1; 
    }
    cout << " " << endl;
    cout << "next_state " << candidate_states[i] << "/goal_lane " << goal_lane << endl;
    
    double speed_up = 0.224;
    // acceleration
    Trajectory candidate_trajectory_acc = splineTrajectoryGen(best_trajectory.ref_vel + speed_up, goal_lane);
    if (candidate_trajectory_acc.next_x_vals.size()!=0) {
      cout << "*************acc*************" << endl;      
      final_trajectories.push_back(candidate_trajectory_acc);
      cost = calculateCost(candidate_trajectory_acc);
      costs.push_back(cost);

    }

    // deceleration
    Trajectory candidate_trajectory_dec = splineTrajectoryGen(best_trajectory.ref_vel - speed_up, goal_lane);
    if (candidate_trajectory_dec.next_x_vals.size()!=0) {
      cout << "*************dec*************" << endl;   
      final_trajectories.push_back(candidate_trajectory_dec);
      cost = calculateCost(candidate_trajectory_dec);
      costs.push_back(cost);
    }
  }

  auto best_cost = std::min_element(costs.cbegin(), costs.cend());
  int best_idx = std::distance(costs.cbegin(), best_cost);  
  best_trajectory = std::move(final_trajectories[best_idx]);
  cout << "best_vel/ " << best_trajectory.ref_vel << endl;
}

vector<string> Vehicle::successorStates(){
  vector<string> candidate_states;
  if (car_d < 4){
    candidate_states.push_back("KL");
    candidate_states.push_back("LCR");
  }
  else if (car_d >= 8) {
    candidate_states.push_back("KL");
    candidate_states.push_back("LCL");	
  }
  else {
    candidate_states.push_back("KL");
    candidate_states.push_back("LCL");
    candidate_states.push_back("LCR");	 	
  }
  return candidate_states;
}

Trajectory Vehicle::splineTrajectoryGen(double goal_vel, double goal_lane) {
  Trajectory spline_trajectory;
  spline_trajectory.ref_vel = goal_vel;
  spline_trajectory.dist_lane_change = std::abs(2 + 4 * goal_lane-car_d);

  // define container for spline curve anchor points
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);


  // if no previous path
  if(prev_size < 2){
  	double prev_car_x = car_x - cos(car_yaw);
  	double prev_car_y = car_y - sin(car_yaw);

  	ptsx.push_back(prev_car_x);
  	ptsx.push_back(car_x);

  	ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);

  }

  // if has previous path
  else {
  	ref_x = previous_path_x[prev_size-1];
  	ref_y = previous_path_y[prev_size-1];
	
	double ref_x_prev = previous_path_x[prev_size-2];
  	double ref_y_prev = previous_path_y[prev_size-2];
  	ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
   
  	ptsx.push_back(ref_x_prev);
  	ptsx.push_back(ref_x);

  	ptsy.push_back(ref_y_prev);
	ptsy.push_back(ref_y);
  }
  

  // add long-distance anchor points
  vector<double> next_wp0 = getXY(car_s + 50, 2+4*goal_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s + 70, 2+4*goal_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s + 90, 2+4*goal_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);


  // transform to vehicle coordinates system
  for(int i =0; i<ptsx.size();++i){
  	double shift_x = ptsx[i]-ref_x;
  	double shift_y = ptsy[i]-ref_y;

  	ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
  	ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
  }

  tk::spline s;
  s.set_points(ptsx,ptsy);

  spline_trajectory.next_x_vals = previous_path_x;
  spline_trajectory.next_y_vals = previous_path_y;

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(pow(target_x,2)+pow(target_y,2));

  double x_add_on = 0;

  for (int i =1; i<= (horizon*50-prev_size);++i) {

  	double N = (target_dist/(.02*goal_vel/2.24));
  	double x_point = x_add_on + target_x/N;
  	double y_point = s(x_point);

  	x_add_on = x_point;

  	double x_ref = x_point;
  	double y_ref = y_point;

  	x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
  	y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

  	x_point += ref_x;
  	y_point += ref_y;

  	spline_trajectory.next_x_vals.push_back(x_point);
  	spline_trajectory.next_y_vals.push_back(y_point);
  }

  return spline_trajectory;
}

double Vehicle::calculateCost(const Trajectory &candidate_trajectory) {

  double cost_speed;
  double cost_change_lane;
  double cost_obstacle_avoidance;
  double cost_traffic_jam;
  vector<double> weights = {1, 0, 1, 10};

  // cost_speed
  cost_speed = calculateCostSpeed(candidate_trajectory);
  cout << "cost_speed " << cost_speed << endl;

  // cost_change_lane
  cost_change_lane = calculateCostChangeLane(candidate_trajectory);
  cout << "cost_change_lane " << cost_change_lane << endl;  

  // cost_safe_distance
  cost_obstacle_avoidance = calculateCostObstacleAvoidance(candidate_trajectory);
  cout << "cost_obstacle_avoidance " << cost_obstacle_avoidance << endl; 

  // cost_traffic_jam
  cost_traffic_jam = calculateCostTrafficJam(candidate_trajectory);
  cout << "cost_traffic_jam " << cost_traffic_jam << endl;

  // weighted_cost 
  double weighted_cost = cost_speed * weights[0] + cost_change_lane * weights[1] + cost_obstacle_avoidance * weights[2] + cost_traffic_jam * weights[3];
  cout << "weighted_cost " << weighted_cost << endl;
  return weighted_cost;

}

double Vehicle::calculateCostSpeed(const Trajectory &candidate_trajectory) {
  double cost_speed;
  double max_speed = 49.5;
  cost_speed = 1 - exp(- (max_speed - candidate_trajectory.ref_vel)/max_speed);
  if (candidate_trajectory.ref_vel >= max_speed) {
   cost_speed = 2;
  }
  return cost_speed;
}

double Vehicle::calculateCostChangeLane(const Trajectory &candidate_trajectory) {
  double cost_change_lane;
  cost_change_lane = 1 - exp(-candidate_trajectory.dist_lane_change / 4);
  return cost_change_lane;
}

double Vehicle::calculateCostObstacleAvoidance(const Trajectory &candidate_trajectory) {
  double cost_safe_distance = 0.0;
  double safe_distance = 3.0;
  bool has_collision = false;
  for (int i = 0; i < horizon * 50; ++i) {
    double candidate_x = candidate_trajectory.next_x_vals[i];
    double candidate_y = candidate_trajectory.next_y_vals[i];  
    for (int j = 0; j < prediction.size(); ++j) {
      if(distance(candidate_x, candidate_y, prediction[j].predicted_x[i], prediction[j].predicted_y[i]) <= safe_distance) {
        has_collision = true;
        cost_safe_distance = 100.0;
        break;
      }
    }  
    if (has_collision == true) {
    	break;
    }
  }
  return cost_safe_distance;
}

double Vehicle::calculateCostTrafficJam(const Trajectory &candidate_trajectory) {
  double cost_traffic_jam; 
  int candidate_trajectory_size = candidate_trajectory.next_x_vals.size();
  double candidate_end_x = candidate_trajectory.next_x_vals[candidate_trajectory_size-1];
  double candidate_end_y = candidate_trajectory.next_y_vals[candidate_trajectory_size-1];
  double candidate_end_prev_x = candidate_trajectory.next_x_vals[candidate_trajectory_size-2];
  double candidate_end_prev_y = candidate_trajectory.next_y_vals[candidate_trajectory_size-2];
  double theta = atan2(candidate_end_y - candidate_end_prev_y, candidate_end_x - candidate_end_prev_x);

  vector<double> candidate_end_frenet = getFrenet(candidate_end_x, candidate_end_y, theta, map_waypoints_x, map_waypoints_y);
  double candidate_end_s = candidate_end_frenet[0];
  double candidate_end_d = candidate_end_frenet[1];
  double candidate_end_lane = floor(candidate_end_d/4);
  
  double distance_ahead = 99999.0;
  for (int i = 0; i < prediction.size() ; ++i) {
    if( prediction[i].final_d<(2+4*candidate_end_lane+2) && prediction[i].final_d >(2+4*candidate_end_lane-2)) {
  	  if ((prediction[i].final_s > candidate_end_s) && (prediction[i].start_s > car_s)) {
        if ((prediction[i].final_s-candidate_end_s) < distance_ahead) {
        	distance_ahead = prediction[i].final_s - candidate_end_s;
        }      
      }
  	}
  }

  cout << "distance_ahead " << distance_ahead << endl;
  double following_min_distance = 5;
  double following_distance = 10;
  if (distance_ahead > following_distance) {
    cost_traffic_jam = 0;
  }
  else {
    cost_traffic_jam = 1.0/pow(following_min_distance - following_distance, 2) * pow(distance_ahead - following_distance, 2);
  }


  return cost_traffic_jam;
}