# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone the repo.
2. Make a build directory: `mkdir build && cd build`
3. Make sure you have a C++14 compliant compiler.
4. Compile: `cmake .. && make`
5. Run it: `./path_planning`.
6. **WARNING**: It is best to redirect stdout to a log file, because the program outputs a lot of debugging info. So, best: `./path_planning > log.txt`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in Frenet coordinates

["d"] The car's d position in Frenet coordinates

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

1. The car uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner receives should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 7.3
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

## File structure

- main.cpp contains the code to receive JSON event messages from the simulator, calls the prediction and behaviour code to calculate new or extended trajectories for the car to follow.
- telemetry_helpers.h/cpp contains the code to extract data wrt the ego car state and other cars (sensor fusion) from the JSON event messages
- conversion.h/cpp contains the code to convert from:
  - degrees to radians and vice versa
  - map coordinate system to Frenet coordinate system and vice versa
  - miles per hour to meter per second and vice versa
- car_state.h/cpp contains a helper structure for static and kinetic info of a particular car
- cost_functions.h/cpp contains several cost functions (most of them already implemented during the course)
- predict.h/cpp contains the code to make predictions of the dynamic behaviour of other cars. This is a very simple implementation based on a model where each car follows its lane with a fixed speed.
- behaviour.h/cpp contains the code that decides what the car should do based on the current state of behaviour mechanism (finite state machine), the state of the car and the predictions of other cars made by the predict module 
- trajectory_generation.h/cpp contains the code to construct trajectories for lane changes:
  - JMT (Jerk minimising trajectories)
  - spline based trajectories (following a given lane)
- third parties code:
  - spline.h : header only library for cubic splines
  - json.hpp: header only library to parse JSON messages
  - Eigen library: matrix library for QR decomposition (to solve linear equations)

## Overview of the code

Repeat while getting JSON event messages containing simulator data:

1. Parse the JSON event messages to retrieve:
   1. the part of the trajectory that was already calculated in previous cycles and has not been traversed
   2. sensor fusion data of other cars
2. Predict the behaviour of other cars (see predict.cpp)
3. Determine if the current manoeuvre of the car should be continued or which manoeuvre the car should execute based on the car predictions and the current car's state. The manoeuvres are implemented as a finite state machine. The states are FOLLOWING_LANE, CHANGE_LANE and SLOW_DOWN_FOR_LANE_CHANGE (self explanatory).
4. If the state is kept, then also the pre-calculated trajectory is not recalculated, but simply extended. If a state changes, then a recalculation of the trajectory is triggered, with an exception of the first points which are kept for smoothness and to take simulator latency into account. 
5. Wrap the recalculated or extended trajectory back into a JSON message and send it back to JSON

## Behaviour model

The behaviour model always starts in the FOLLOWING_LANE state.

### FOLLOWING_LANE state

- If the car up front is closer than a safety distance, then we should recalculate the trajectory taking a velocity change into account. **SAFETY FIRST !**
- If we detected that there is a faster lane (based on other car's predictions and the ego car state), then we should consider a lane change:
  - We always look for a single lane change in the direction of the fastest lane: we don't try to be a pirate.
  - If there are cars up front and behind in the target lane, then we see if minimum distances can be kept. If so, then we recalculate the trajectory so that a lane change can be executed and change to the CHANGE_LANE state.
  - If not so, then we recalculate the trajectory to slow down for the car up front or behind in the target lane and change to the SLOW_DOWN_FOR_LANE_CHANGE state.
  - Take into account that the notion of "a faster lane" is based on the velocities of the cars in a certain horizon. The reason why the car doesn't change lanes even if there is a gap to jump into, is because the behaviour model knows there is no use to change the lanes, since the car up front is slower.
- Else (if there is no faster lane):
  - If the space in front of the ego car is larger than a fixed value:
    - If the target speed is smaller than the ideal speed, then recalculate the trajectory to accelerate to the ideal speed. The ideal speed is chosen to be a bit lower than the speed limit. Acceleration and deceleration is seen as "re-entering" the FOLLOWING_LANE state.
    - If the ideal speed is met, the pre-calculated trajectory is extended.
  - Else if the current target speed has a delta (a certain margin) larger than the velocity of the car up front, then recalculate the trajectory (accelerate or decelerate) with a modified target velocity.
  - Else: Just extend the trajectory using the same target velocity.

### CHANGE_LANE state

- If the car up front in the target lane for a lane change changed drastically (above a given margin), then we recalculate the "lane change" trajectory with a new target speed and new target position. This can be regarded as "re-entering" the CHANGE_LANE state.
- Else, if we past the target position (s coordinate in the Frenet coordinate system), then we go back to the FOLLOWING_LANE state.
- Else (still performing the lane change), the just extend the trajectory following the target lane of the lane change.

### SLOW_DOWN_FOR_LANE_CHANGE state

- If there is still a faster lane (reason why we slowed down !):
  - If we are still not enough behind the car in the target lane that we slowed down for, then extend the pre-calculated trajectory (containing the deceleration) following the current lane
  - Else: the car could consider a lane change. See the rules of the FOLLOWING_LANE state to determine if we can perform a lane change or not and to which state the behaviour model should migrate.
- Else (no faster lane anymore):
  - Change the FOLLOWING_LANE state.

## Trajectory generation

Two types of trajectory generation methods are implemented:

- JMT: this method is used when a lane change is needed. Note that the JMT uses the map coordinates instead of the Frenet coordinate system, due to the fact the Frenet coordinates are not converted well to map coordinates: the lack of continuity in converting subsequent (s,d) coordinates to map coordinates causes problems with jerk and speed limits. Only the (s,d) coordinates are used to calculate the heading of the car and the direction of the velocity vector.
- Spline based trajectory: used to extend an existing trajectory or calculate a trajectory where a velocity change is needed in the same lane. This makes sure that the lane is followed smoothly.

## Possible improvements

- Generate multiple short term trajectories and let the implemented cost functions decide which one is best.
- Decision making in the behaviour model could still be improved:
  - The safety distances could for example be adapted to the current speed and the speed of the other cars.
  - Only data from the car up front and behind in each lane are taken into account. In many situations, this is enough, but sometimes it useful to know that other cars more up front are blocking the car up front.
  - Sometimes if the road is blocked by slower cars in all the lanes, then the behaviour model changes lane quite often. This is due to the fact that the cars up front sometimes accelerate or decelerate just briefly: this causes the behaviour model to consider a lane change. 
- The conversion in between Frenet and map coordinates could be improved. Even with the offered solution in the Udacity Waffle, the conversion caused a lot of issues. A more granular Frenet grid of waypoints could improve the situation.

## Demo

A small video can be found in the repository. A situation is depicted where a car is following the car up front  (in the same lane) until the situation is reached where this car can be passed due to the fact that the minimum distance with the car in the back in the target lane is exceeded (and thus it is safe to change the lane).

See: self_driving_car_nanodegree_program 03_08_2018 01_54_09.mp4