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
best_trajectory.action = "KL";
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
  double following_distance = 30.0;
  double vel_ahead = 999.0;
  bool has_car_ahead = false;
  // generate motion prediction for vehicles around the main vehicle
  for (int i = 0; i< sensor_fusion.size(); ++i) {
    double detected_car_x = sensor_fusion[i][1];
    double detected_car_y = sensor_fusion[i][2];
    if (distance(car_x, car_y, detected_car_x, detected_car_y) < 120.0) {
      PredictedTrajectory check_prediction;
      check_prediction.id = sensor_fusion[i][0];
  	  double vx = sensor_fusion[i][3];
  	  double vy = sensor_fusion[i][4];
  	  double check_car_s = sensor_fusion[i][5];  
  	  float check_car_d = sensor_fusion[i][6];
  	  double check_speed = sqrt(vx*vx + vy*vy);

 
	  double car_lane = floor(car_d / 4.0);
	    if ((check_car_d < (2 + 4 * car_lane + 2)) && (check_car_d > (2 + 4 * car_lane - 2))) {
	    // ignore the car right behind
        if (check_car_s < car_s) {
          continue;
        }
          
          // check the nearest car ahead
		  if ((check_car_s > car_s) && ((check_car_s - car_s) < following_distance)) {
			  following_distance = check_car_s - car_s;
			  vel_ahead = check_speed;
			  has_car_ahead = true;
	    }
	  }

      for(int i = 0; i < horizon*50; ++i) {
        double check_car_s_predict = check_car_s + (i * 0.02 * check_speed);
				check_prediction.predicted_s.push_back(check_car_s_predict);
				check_prediction.predicted_d.push_back(check_car_d);
      }
      prediction.push_back(check_prediction);
    }
  }

  // update max_vel
  cout << "**********************************************" << endl;
  if (has_car_ahead == true) {
  	cout << "following distance " << following_distance << endl;
  }
  else {
  	cout << "no car ahead within " << following_distance << " meters" << endl;
  }

  max_vel = 22.0;
  if ((has_car_ahead == true) && (best_trajectory.action == "KL")) {
    max_vel = vel_ahead - 2;
  }	  

  if (following_distance < 10) {
  	max_vel = vel_ahead - 5;
  }
 
  cout << "car_d " << car_d << endl; 

  // generate distance of car ahead on each lane
  double sensing_distance = 120.0;
  vector<double> new_distances_ahead = {sensing_distance, sensing_distance, sensing_distance};
  distances_ahead = std::move(new_distances_ahead);

  for (int i = 0; i < prediction.size() ; ++i) {

  	// update lane 0 
    if( prediction[i].predicted_d[0]< 4 && prediction[i].predicted_d[0] >= 0) {
  	  if (prediction[i].predicted_s[0] > car_s) {
        if ((prediction[i].predicted_s[0] - car_s) < distances_ahead[0]) {
        	distances_ahead[0] = prediction[i].predicted_s[0] - car_s;
        }      
      }
  	}

  	// update lane 1 
    if( prediction[i].predicted_d[0]< 8 && prediction[i].predicted_d[0] >= 4) {
  	  if (prediction[i].predicted_s[0] > car_s) {
        if ((prediction[i].predicted_s[0] - car_s) < distances_ahead[1]) {
        	distances_ahead[1] = prediction[i].predicted_s[0] - car_s;
        }      
      }
  	}

  	// update lane 2
    if( prediction[i].predicted_d[0] <= 12 && prediction[i].predicted_d[0] >= 8) {
  	  if (prediction[i].predicted_s[0] > car_s) {
        if ((prediction[i].predicted_s[0] - car_s) < distances_ahead[2]) {
        	distances_ahead[2] = prediction[i].predicted_s[0] - car_s;
        }      
      }
  	}
  }
}

void Vehicle::FSMPlanner() {

  vector<string> candidate_states = successorStates();
  float cost;
  vector<float> costs;
  vector<Trajectory> final_trajectories;
  double current_lane = floor(car_d/4);
  
  for(int i = 0; i < candidate_states.size(); ++i) {
  	vector<double> anchor_waypoint_lane;
  	vector<double> anchor_waypoint_s;
    if (candidate_states[i] == "KL") {
    	anchor_waypoint_lane = {current_lane, current_lane, current_lane};
    	if (ref_vel < 5) {
				anchor_waypoint_s = {30.0, 60.0, 90.0};
    	}
    	else {
    		anchor_waypoint_s = {ref_vel*1.5, ref_vel*3.0, ref_vel*4.5};
    	}  	
    }
    else if(candidate_states[i] == "LCL") { 
    	anchor_waypoint_lane = {current_lane-1, current_lane-1, current_lane-1};
    	if (ref_vel < 5) {
				anchor_waypoint_s = {60.0, 80.0, 100.0}; 
    	}
    	else {    	
    		anchor_waypoint_s = {ref_vel*2.5, ref_vel*4.0, ref_vel*5.0};
    	}
    }
    else if (candidate_states[i] == "LCR") {
    	anchor_waypoint_lane = {current_lane+1, current_lane+1, current_lane+1};
    	if (ref_vel < 5) {
    	  anchor_waypoint_s = {60.0, 80.0, 100.0}; 
    	}
    	else {
    	  anchor_waypoint_s = {ref_vel*2.5, ref_vel*4.0, ref_vel*5.0};   		
    	}
    }
    else if (candidate_states[i] == "PLCR") {
    	anchor_waypoint_lane = {current_lane+1, current_lane+1, current_lane+2};
    	if (ref_vel < 5) {
    	  anchor_waypoint_s = {60.0, 80.0, 120.0};
    	}
    	else {
			  anchor_waypoint_s = {ref_vel*2.5, ref_vel*4.0, ref_vel*6.0};
    	}
    }
    else if (candidate_states[i] == "PLCL") {
    	anchor_waypoint_lane = {current_lane-1, current_lane-1, current_lane-2};
    	if (ref_vel < 5) {
    	  anchor_waypoint_s = {60.0, 80.0, 120.0};  
    	}
    	else {
    	  anchor_waypoint_s = {ref_vel*2.5, ref_vel*4.0, ref_vel*6.0};
    	}
    }

    cout << " " << endl;
    cout <<  candidate_states[i] << endl;
    
    Trajectory candidate_trajectory = splineTrajectoryGen(ref_vel, candidate_states[i], anchor_waypoint_lane, anchor_waypoint_s);
    if (candidate_trajectory.next_x_vals.size()!=0) {  
      cost = calculateCost(candidate_trajectory);    
      if ((cost < 99999.0) || (candidate_states[i] == "KL")) {
        final_trajectories.push_back(candidate_trajectory);
        costs.push_back(cost);     	
      }
    }
  }
  
  // if the only trajectory has collision which indicates emergency, set max_vel to 0
  if ((costs.size() == 1) && (costs[0] > 99998.0)) {
  	max_vel = 5.0;
  }
  
  auto best_cost = std::min_element(costs.cbegin(), costs.cend());
  int best_idx = std::distance(costs.cbegin(), best_cost);  
  best_trajectory = std::move(final_trajectories[best_idx]);

  cout << " " << endl;  
  cout << "action: " << best_trajectory.action << endl;
  
  // update ref_vel
  if (ref_vel < max_vel) {
	  ref_vel += 0.2;
  }
  else if (ref_vel > max_vel) {
	  ref_vel -= 0.2;
  }

  cout << "max velocity " << max_vel << endl;
  cout << "ref velocity " << ref_vel << endl;
}

vector<string> Vehicle::successorStates(){
  vector<string> candidate_states;
  if (car_d < 4){
    candidate_states.push_back("KL");
    candidate_states.push_back("LCR");
    candidate_states.push_back("PLCR");    
  }
  else if (car_d >= 8) {
    candidate_states.push_back("KL");
    candidate_states.push_back("LCL");
    candidate_states.push_back("PLCL"); 	
  }
  else {
    candidate_states.push_back("KL");
    candidate_states.push_back("LCL");
    candidate_states.push_back("LCR");	 	
  }

  vector<string> feasible_candidate_states;
  // if the previous action is LCL, then LCR and PLCR are not allowed
  if (best_trajectory.action == "LCL") {
		for (int i = 0; i<3 ;++i) {
			if ((candidate_states[i] != "LCR") && (candidate_states[i] != "PLCR")) {
				feasible_candidate_states.push_back(candidate_states[i]);
			}
  	}
  	candidate_states = feasible_candidate_states;
  }

  // if the previous action is PLCL, then LCR and PLCR are not allowed
  if (best_trajectory.action == "PLCL") {
		for (int i = 0; i<3 ;++i) {
			if ((candidate_states[i] != "LCR") && (candidate_states[i] != "PLCR")) {
				feasible_candidate_states.push_back(candidate_states[i]);
			}
  	}
  	candidate_states = feasible_candidate_states;
  }

  // if the previous action is LCR, then LCL and PLCL are not allowed
  if (best_trajectory.action == "LCR") {
		for (int i = 0; i<3 ;++i) {
			if ((candidate_states[i] != "LCL") && (candidate_states[i] != "PLCL")) {
				feasible_candidate_states.push_back(candidate_states[i]);
			}
  	}
  	candidate_states = feasible_candidate_states;
  }

  // if the previous action is PLCR, then LCL and PLCL are not allowed
  if (best_trajectory.action == "PLCR") {
		for (int i = 0; i<3 ;++i) {
			if ((candidate_states[i] != "LCL") && (candidate_states[i] != "PLCL")) {
				feasible_candidate_states.push_back(candidate_states[i]);
			}
  	}
  	candidate_states = feasible_candidate_states;
  }

  return candidate_states;
}

Trajectory Vehicle::splineTrajectoryGen(const double goal_vel, const string action, const vector<double> &anchor_waypoint_lane, const vector<double> &anchor_waypoint_s) {
  Trajectory spline_trajectory;
  spline_trajectory.action = action;
  spline_trajectory.target_lane = anchor_waypoint_lane.back();

  // define container for spline curve anchor points
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  // if no previous path
  if(prev_size < 2){
  	double prev_car_x = car_x - cos(ref_yaw);
  	double prev_car_y = car_y - sin(ref_yaw);

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
  vector<double> next_wp0 = getXY(car_s + anchor_waypoint_s[0], 2+4*anchor_waypoint_lane[0], map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s + anchor_waypoint_s[1], 2+4*anchor_waypoint_lane[1], map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s + anchor_waypoint_s[2], 2+4*anchor_waypoint_lane[2], map_waypoints_s, map_waypoints_x, map_waypoints_y);

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

  for (int i = 1; i<= (horizon*50-prev_size);++i) {

  	double N = (target_dist/(.02*goal_vel));
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

  // convert x,y trajectory to s,d trajectory
  double theta;
  for (int i = 0; i < horizon * 50 - 1; ++i) {
	  double spline_x = spline_trajectory.next_x_vals[i];
	  double spline_y = spline_trajectory.next_y_vals[i];
	  double spline_next_x = spline_trajectory.next_x_vals[i+1];
	  double spline_next_y = spline_trajectory.next_y_vals[i + 1];
	  theta = atan2(spline_next_y - spline_y, spline_next_x - spline_x);
	  vector<double> spline_frenet = getFrenet(spline_x, spline_y, theta, map_waypoints_x, map_waypoints_y);
	  spline_trajectory.next_s_vals.push_back(spline_frenet[0]);
	  spline_trajectory.next_d_vals.push_back(spline_frenet[1]);
  }
  vector<double> spline_end_frenet = getFrenet(spline_trajectory.next_x_vals.back(), spline_trajectory.next_y_vals.back(), theta, map_waypoints_x, map_waypoints_y);
  spline_trajectory.next_s_vals.push_back(spline_end_frenet[0]);
  spline_trajectory.next_d_vals.push_back(spline_end_frenet[1]);

  return spline_trajectory;
}

/*
Trajectory Vehicle::quinticPolynomialTrajectoryGen(double goal_vel, double goal_lane) {
	Trajectory quinticPolynomial;
	return quinticPolynomial;
}
*/

double Vehicle::calculateCost(const Trajectory &candidate_trajectory) {

  double cost_obstacle_avoidance;
  double cost_traffic_jam;
  vector<double> weights = {1, 3};

  // cost_obstacle_avoidance
  cost_obstacle_avoidance = weights[0] * calculateCostObstacleAvoidance(candidate_trajectory);
  cout << "cost_obstacle_avoidance " << cost_obstacle_avoidance << endl; 

  // cost_traffic_jam

  cost_traffic_jam = weights[1] * calculateCostTrafficJam(candidate_trajectory);
  cout << "cost_traffic_jam " << cost_traffic_jam << endl;

  // weighted_cost 
  double weighted_cost = cost_obstacle_avoidance + cost_traffic_jam;
  cout << "weighted_cost " << weighted_cost << endl;
  return weighted_cost;

}

double Vehicle::calculateCostObstacleAvoidance(const Trajectory &candidate_trajectory) {
  double cost_safe_distance = 0.0;
  double safe_lateral_distance = 3.0;
  double safe_longitudinal_distance = 6.0;
  bool has_collision = false;
  for (int i = 0; i < horizon * 50; ++i) {
    double candidate_s = candidate_trajectory.next_s_vals[i];
    double candidate_d = candidate_trajectory.next_d_vals[i];  
    for (int j = 0; j < prediction.size(); ++j) {
      if((candidate_d <= (prediction[j].predicted_d[i] + safe_lateral_distance)) && 
		  (candidate_d >= (prediction[j].predicted_d[i] - safe_lateral_distance)) && 
          (candidate_s <= (prediction[j].predicted_s[i] + safe_longitudinal_distance)) &&
		  (candidate_s >= (prediction[j].predicted_s[i] - safe_longitudinal_distance))) {
        has_collision = true;
        cost_safe_distance = 99999.0;  // penalty for collision
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
  double sensing_distance = 120.0;
  double cost_traffic_jam; 
  double candidate_end_lane = candidate_trajectory.target_lane;
  
  vector<double> fuzzy_distances_ahead = fuzzyDistance(distances_ahead);
  
  double fuzzy_distance = fuzzy_distances_ahead[(int)candidate_end_lane];
  cout << "fuzzy_distance_ahead " << fuzzy_distance << endl;
  cost_traffic_jam = 1.0/(sensing_distance * sensing_distance) * (fuzzy_distance - sensing_distance) * (fuzzy_distance - sensing_distance);

  return cost_traffic_jam;
}

/*
double Vehicle::calculateCostComfort(const Trajectory &candidate_trajectory) {
  double cost_comfort = 0.0;
  double delta_t = 0.02;
  double acc_lim = 9.5;
  double jerk_lim = 9.5;
  for(int i = 0; i< candidate_trajectory.next_s_vals.size()-3; ++i) {
  	// calculate s acc and jerk
    double s_i_0 = candidate_trajectory.next_s_vals[i];
    double s_i_1 = candidate_trajectory.next_s_vals[i+1];
    double s_i_2 = candidate_trajectory.next_s_vals[i+2];
    double s_i_3 = candidate_trajectory.next_s_vals[i+3];

    double vs_i_0 = (s_i_1-s_i_0)/delta_t;
    double vs_i_1 = (s_i_2-s_i_1)/delta_t;
	  double vs_i_2 = (s_i_3-s_i_2)/delta_t;

    double as_i_0 = (vs_i_1-vs_i_0)/delta_t;
    double as_i_1 = (vs_i_2-vs_i_1)/delta_t;

    double js_i_0 = (as_i_1-as_i_0)/delta_t;

  	// calculate d acc and jerk
    double d_i_0 = candidate_trajectory.next_d_vals[i];
    double d_i_1 = candidate_trajectory.next_d_vals[i+1];
    double d_i_2 = candidate_trajectory.next_d_vals[i+2];
    double d_i_3 = candidate_trajectory.next_d_vals[i+3];

    double vd_i_0 = (d_i_1-d_i_0)/delta_t;
    double vd_i_1 = (d_i_2-d_i_1)/delta_t;
	  double vd_i_2 = (d_i_3-d_i_2)/delta_t;

    double ad_i_0 = (vd_i_1-vd_i_0)/delta_t;
    double ad_i_1 = (vd_i_2-vd_i_1)/delta_t;

    double jd_i_0 = (ad_i_1-ad_i_0)/delta_t;  	

    if (i==0) {
		  cout << "acc_s " << as_i_0  << endl;		
		  cout << "acc_d " << ad_i_0  << endl;			    
		  cout << "acc " << sqrt(as_i_0 * as_i_0 + ad_i_0 * ad_i_0) << endl;
		  cout << "jerk" << sqrt(js_i_0 * js_i_0 + jd_i_0 * jd_i_0) << endl;    	
    }

    // calculate cost
    // || (sqrt(js_i_0 * js_i_0 + jd_i_0 * jd_i_0) > jerk_lim)
    if (sqrt(as_i_0 * as_i_0 + ad_i_0 * ad_i_0) > acc_lim) {
		  cost_comfort= 99.0;
		  break;
    }
  }
  return cost_comfort;
}
*/