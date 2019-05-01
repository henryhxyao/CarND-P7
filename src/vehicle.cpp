#include <vector>
#include "vehicle.h"
#include "helpers.h"
#include "spline.h"
using std::vector;
#include <iostream>

Vehicle::Vehicle(const vector<double> &input_map_waypoints_x,
				const vector<double> &input_map_waypoints_y,
				const vector<double> &input_map_waypoints_s,
				const vector<double> &input_map_waypoints_dx,
				const vector<double> &input_map_waypoints_dy,
				const double &input_ref_vel,
				const int &input_lane) {

map_waypoints_x = std::move(input_map_waypoints_x);
map_waypoints_y = std::move(input_map_waypoints_y);
map_waypoints_s = std::move(input_map_waypoints_s);
map_waypoints_dx = std::move(input_map_waypoints_dx);
map_waypoints_dy = std::move(input_map_waypoints_dy);
ref_vel = input_ref_vel;
lane = input_lane;

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

void Vehicle::FSMPlanner(const vector<vector<double>> &sensor_fusion) {
  if(prev_size > 0) {
    car_s = end_path_s;
  }

  bool too_close = false;

  for (int i = 0; i< sensor_fusion.size(); ++i) {

  	float d = sensor_fusion[i][6];
  	if( d<(2+4*lane+2) && d>(2+4*lane-2)) {
  	  double vx = sensor_fusion[i][3];
  	  double vy = sensor_fusion[i][4];
  	  double check_speed = sqrt(vx*vx + vy*vy);
  	  double check_car_s = sensor_fusion[i][5];

  	  check_car_s += ((double)prev_size*.02*check_speed);

  	  if ((check_car_s > car_s) && ((check_car_s-car_s)<30)) {
  		  too_close = true;
  		  if(lane > 0) {
  		  	lane = 0;
  		  }
  	  }
    }
  }

  if (too_close) {

  	ref_vel -= .224;
  }
  else if (ref_vel < 49.5) {
  	ref_vel += .224;
  }
}

bool Vehicle::splineTrajectoryGen(vector<double> &next_x_vals, vector<double> &next_y_vals) {
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
  vector<double> next_wp0 = getXY(car_s + 30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s + 60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s + 90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

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

	for (int i =0; i< previous_path_x.size(); i++){
	  next_x_vals.push_back(previous_path_x[i]);
	  next_y_vals.push_back(previous_path_y[i]);
	}

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(pow(target_x,2)+pow(target_y,2));

  double x_add_on = 0;

  for (int i =1; i<= 50-previous_path_x.size();++i) {

  	double N = (target_dist/(.02*ref_vel/2.24));
  	double x_point = x_add_on + target_x/N;
  	double y_point = s(x_point);

  	x_add_on = x_point;

  	double x_ref = x_point;
  	double y_ref = y_point;

  	x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
  	y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

  	x_point += ref_x;
  	y_point += ref_y;

  	next_x_vals.push_back(x_point);
  	next_y_vals.push_back(y_point);
  }

  if (next_x_vals.size() != 0) {
    return 0;
  }
  else {
    return 1;
  }
}
