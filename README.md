# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

## Model Explanation
[image1]: ./img/result.png "best case"

### Overview

A class named `Vehicle` (which is defined in "vehicle.cpp" and "vehicle.h") is created to store all the useful information and define all the behaviors of the main vehicle.

Then in the main(), the following steps are taken: 
1. an Vehicle object `agent` is declared and the map_waypoints info is stored in `agent`. 

"main.cpp" line 57:
```c++
Vehicle agent(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
```
2. After the main car's localization and sensor fusion data are received, store these data in `agent`.

"main.cpp" line 95:
```c++
agent.setState(j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], j[1]["speed"], j[1]["previous_path_x"], j[1]["previous_path_y"], j[1]["end_path_s"], j[1]["end_path_d"]);
```
3. Using the sensor fusion data, the `agent` predicts the trajectory of other vehicles in the time horizon of 3s.

"main.cpp" line 103:
```c++
agent.generatePrediction(sensor_fusion);
```
4. The `agent` generate the best trajectory of time horizon = 3s  

"main.cpp" line 105:
```c++
agent.FSMPlanner();
```
5. Only output the first 0.5 second trajectory to the simulator and throw away the next 2.5 seconds motion in the best_trajectory.

"main.cpp" line 108-109:
```c++
vector<double> next_x_vals(agent.best_trajectory.next_x_vals.cbegin(), agent.best_trajectory.next_x_vals.cbegin() + 25);
vector<double> next_y_vals(agent.best_trajectory.next_y_vals.cbegin(), agent.best_trajectory.next_y_vals.cbegin() + 25);
```

### Prediction

#### generate prediction: **"vehicle.cpp" line 53-94**
A simple linear model (the vehicle's speed and lateral positon check_car_d are constant) is used to generate motion prediction, `predicted_s` and `predicted_d` for the vehicles within 120m around the main vehicle. Meanwhile, check if there is a vehicle on the same lane with the main vehicle within the distance of 30m. If so, record its distance to the main vehicle as `following_distance` and its velocity as `vel_ahead`.

#### update max velocity: **"vehicle.cpp" line 96-114**
1. set max velocity `max_vel` to be 22m/s (equals to 49.2mph which is within speed limit);
2. if `following_distance` is less than 30m and the current trajectory is keeping lane, then set `max_vel` to be `vel_ahead`-2; 
3. if `following_distance` is less than 10m which indicates potential danger, then set `max_vel` to be `vel_ahead`-5;
2 and 3 are used to keep enough space between the vehicle ahead.

#### generate distance_ahead on each lane: **"vehicle.cpp" line 116-150**
To measure which lane is currently free of traffic, calculate the longitudinal distance of the nearest vehicle ahead on each lane (lane0, lane1, lane2) to the main vehicle and store these three distances as `distances_ahead`. These values are used in the cost calculation function.

### Planner
#### generate successor states:
1. A finite state machine of 5 states are used:
* KL: keep lane;
* LCL: lane change left;
* PLCL: prepare lane change left. the vehicle first change to lane 1, then change to lane 0. Therefore, PLCL can also be called lane change left twice;
* LCR: lane change right;
* PLCR: prepare lane change right. the vehicle first change to lane 1, then change to lane 2. Therefore, PLCR can also be called lane change right twice.

2. Two constraints on the states are used:
* The vehicle cannot choose actions that leads the vehicle out of the lanes: **"vehicle.cpp" line 248-262**

To be specific, when the main vehicle is on lane0, then it can only choose "KL","LCR","PLCR";  when the main vehicle is on lane1, then it can only choose "KL","LCR","LCL"; when the main vehicle is on lane2, then it can only choose "KL","LCL","PLCL"; 

* To avoid the accelerations and jerks of the planned trajectories getting to large, the following state transitions are not allowed: **"vehicle.cpp" line 264-306** 

LCL <-> LCR, LCL <-> PLCR, PLCL <-> LCR, PLCL <-> PLCR

#### trajectory generation
For each state in the successor_states:
1. select anchor points according to the specific state: **"vehicle.cpp" line 163-206**
If the vehicle's velocity `ref_vel` is lower than 5m/s, lane change at fixed distances is used: 

| action | anchor points (s, lane_number) |
| ------ | ------ |
| "KL" | (30, current_lane)<br>(60, current_lane)<br>(90, current_lane)|
| "LCL" | (60, current_lane-1)<br>(80, current_lane-1)<br>(100, current_lane-1)|
| "LCR" | (60, current_lane+1)<br>(80, current_lane+1)<br>(100, current_lane+1)|
| "PLCR" | (60, current_lane+1)<br>(80, current_lane+1)<br>(120, current_lane+2)|
| "PLCL" | (60, current_lane-1)<br>(80, current_lane-1)<br>(120, current_lane-2)|

if the vehicle's velocity `ref_vel` is larger than 5m/s, lane change at fixed time is used: 

| action | anchor points (s, lane_number) |
| ------ | ------ |
| "KL" | (`ref_vel`*1.5, current_lane)<br>(`ref_vel`*3.0, current_lane)<br>(`ref_vel`*4.5, current_lane)|
| "LCL" | (`ref_vel`*2.5, current_lane-1)<br>(`ref_vel`*4.0, current_lane-1)<br>(`ref_vel`*5.0, current_lane-1)|
| "LCR" | (`ref_vel`*2.5, current_lane+1)<br>(`ref_vel`*4.0, current_lane+1)<br>(`ref_vel`*5.0, current_lane+1)|
| "PLCR" | (`ref_vel`*2.5, current_lane+1)<br>(`ref_vel`*4.0, current_lane+1)<br>(`ref_vel`*6.0, current_lane+2)|
| "PLCL" | (`ref_vel`*2.5, current_lane-1)<br>(`ref_vel`*4.0, current_lane-1)<br>(`ref_vel`*6.0, current_lane-2)|

2. use the two points at the end of the previous path as the first two anchor points: **"vehicle.cpp" line 317-361**
3. transform the above anchor points to the vehicle's local coordinate system and generate spline curve `s`: **"vehicle.cpp" line 364-373**
4. combine the previous_path points and `horizon*50-prev_size` new points generated by the spline curve (new points have been transformed back to the global coordinate system) to form the new candidate trajectory: **"vehicle.cpp" line 375-403**
5. convert x,y trajectories to s,d trajectories which are used in collision detect: **"vehicle.cpp" line 406-419**

#### cost calculation
1. cost_obstacle_avoidance: **"vehicle.cpp" line 453-476**
For time horizon = 3s, there are 3 * 50 time substeps. The collision detection is done in frenet coordinates between the main vehicle and any other vehicles in the prediction at each time substep. If the longitudinal distance between the two vehicles is below `safe_longitudinal_distance` = 6.0m, as well as the lateral distance between the two vehicles is below `safe_lateral_distance` = 3.0m, the two vehicles are regarded as having collision with each other.

If there is collision with any vehicle at any time substep for the main vehicle, then the `cost_obstacle_avoidance` = 99999.0 which is a large penalty for collision. Otherwise, the `cost_obstacle_avoidance` = 0.0.

2. cost_traffic_jam: **"vehicle.cpp" line 478-491**
The `distances_ahead`(vector<double>) which has already been calculated in the prediction serves as a good indication of the traffic condition on each lane. If the `distances_ahead[i]` is large, then the i-th lane is good because the vehicle ahead on that lane is quite far away. Therefore, it is obvious to select the lane with the largest value in the `distances_ahead` as the best lane. In terms of `cost_traffic_jam`, the trajectory which end lane's distance_ahead is largest should have the lowest cost value.

However, this cost function design can lead to rapid decision change when any two lanes has similar distance_ahead. For example, if the planner is given the following distances_ahead time series: 

| time | distances_ahead | goal_lane_selection |
| ------ | ------ |------ |
| t0 | {15, 12, 16} | 2 |
| t1 | {15, 12, 14} | 0 |
| t2 | {15, 12, 16} | 2 |
then the main vehicle will drive to the 2-th lane, back to the 0-th lane, and then to the 2-th lane within 2 seconds. This behavior definitely will cause large acceleration and jerk which is not wanted.

Though an additional cost function for comfort can be added to punish the bahavior with quick decision change. Here, I introduce a fuzzyDistance function (**"helpers.h" line 157-194**) to sort of filter the distance_ahead. If the difference between any two distances in distances_ahead is less than 20m, then they are regarded as belonging to the same distance group. The distances in the same distance group are averaged. For example, {15, 12, 16}->{14.33, 14.33, 14.33}, {120, 110, 40}-{115, 115, 40}. I found this method can increase the robustness of the planning algorithm and avoid unnecessary lane change. The philosophy behind this fuzzyDistance function is also similar to human driver's decision making process that the human drivers will wait and not do lane change until the advantage of changing lane is large enough.

After the distance_ahead is transformed into fuzzy_distance, then a parabola cost function is used to calculate the cost.

3. the final cost equals to `weights[0] * cost_obstacle_avoidance + weights[1] * cost_traffic_jam`

#### select the best trajectory and update the max velocity:  **"vehicle.cpp" line 212-217**
For the trajectory to a specific state, if the weighted_cost is not smaller than 99999.0 which indicates collision, then this trajectory is not taken into account, except for that this trajectory is a keep-lane trajectory. Then select the trajectory with the lowest weighted cost as the `best_trajectory`.

if there is only one trajectory that can be selected and also has collision which means an emergency, then the max velocity `max_vel` is set to 5.0 m/s.

#### update reference velocity: **"vehicle.cpp" line 234-240**
If the reference velocity `ref_vel` is lower than max velocity `max_vel`, then add 0.2 to the `ref_vel`; otherwise if the reference velocity `ref_vel` is larger than max velocity `max_vel`, then minus 0.2 from the `ref_vel`;

### Performance
The above proposed planning algorithm achieved a best distance without incident of 61.27miles on my local machine, and average distance between two accidents is about 30 miles. The screenshot of the best test is shown below:
![alt text][image1]

### Further improvements
1. For now, the proposed method will fail if the vehicle on the adjacent lane suddenly changes lane. One solution will be using more complex prediction models like AMM and trajectory clustering introduced in the lessons.
2. The parameters in the algorithms can be further tuned and the speed regulation method can be improved. 

