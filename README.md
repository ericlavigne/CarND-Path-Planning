# Path Planning
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Self-Driving Car Engineer Nanodegree Program

---

Path planning on a highway using A* algorithm for safely passing other cars.

[![video of car on highway](https://github.com/ericlavigne/CarND-Path-Planning/raw/master/img/youtube-thumb.png)](https://youtu.be/ouSjnpnRL7w)

---

### Beyond the Requirements

* A* algorithm for path planning rather than heuristics
* Clear separation of concerns: A* algorithm, trajectory planner, controller, prediction
* Automated tests for the trajectory planner

*Note: Find the latest version of this project on
[Github](https://github.com/ericlavigne/CarND-Path-Planning).*

---

### Contents

* [Project Components](#project-components)
  * [A* Algorithm](#a-star-algorithm)
  * [Prediction](#prediction)
  * [Trajectory State](#trajectory-state)
  * [Discrete Trajectory Planner](#discrete-trajectory-planner)
  * [Automated Testing](#automated-testing)
  * [Trajectory Planner](#trajectory-planner)
  * [Frenet Conversion](#frenet-conversion)
  * [Controller](#controller)
* [Usage](#usage)
  * [Downloading the Simulator](#downloading-the-simulator)
  * [Installing Dependencies](#installing-dependencies)
  * [Build Instructions](#build-instructions)
  * [Running Automated Tests](#running-automated-tests)
  * [Running Planner with the Simulator](#running-planner-with-the-simulator)
* [Background](#background)
  * [Goals](#goals)
  * [Highway Map](#highway-map)
  * [Data from Simulator](#data-from-simulator)
  * [Details](#details)

---

### Project Components

#### A-Star Algorithm

* Generic A* algorithm can work with variety of state representation.
* Applied to Trajectory State for 4-dimensional path planning: s, d, v, t.
* See code in [astar.h](https://github.com/ericlavigne/CarND-Path-Planning/blob/master/src/astar.h).

#### Prediction

* Predicts future positions of other cars by assuming that they will move at constant speed within their current lanes.
* Cars that are changing lanes are conservatively predicted to be in both lanes two seconds later.
* See code in [discrete\_prediction.h](https://github.com/ericlavigne/CarND-Path-Planning/blob/master/src/discrete_prediction.h) and [discrete\_prediction.cpp](https://github.com/ericlavigne/CarND-Path-Planning/blob/master/src/discrete_prediction.cpp).

#### Trajectory State

* Represents 4-dimensional car state (s,d,v,t) for A* algorithm in context of discrete trajectory planner.
* Includes basic car physics: acceleration, deceleration, and lane changing.
* Includes goal of advancing along the highway.
* Applies penalties for crashing or tailgating.
* See code in [trajectory\_state.h](https://github.com/ericlavigne/CarND-Path-Planning/blob/master/src/trajectory_state.h) and [trajectory\_state.cpp](https://github.com/ericlavigne/CarND-Path-Planning/blob/master/src/trajectory_state.cpp).

#### Discrete Trajectory Planner

* Combines A* algorithm with Trajectory State to perform planning.
* Discretization with 1 meter precision along the road and 2 meter precision across the road to reduce A* search space.
* See code in [discrete\_trajectory\_planner.h](https://github.com/ericlavigne/CarND-Path-Planning/blob/master/src/discrete_trajectory_planner.h) and [discrete\_trajectory\_planner.cpp](https://github.com/ericlavigne/CarND-Path-Planning/blob/master/src/discrete_trajectory_planner.cpp).

#### Automated Testing

* Test for optimal behavior in four scenarios:
  * Clear road
  * Passing between two cars
  * All lanes blocked
  * Snaking through formation of cars
* See code in [discrete\_trajectory\_planner\_test.cpp](https://github.com/ericlavigne/CarND-Path-Planning/blob/master/test/discrete_trajectory_planner_test.cpp).

#### Trajectory Planner

* Converts continuous data about self and other cars into discrete representation for Discrete Trajectory Planner.
* Converts discrete representation back to continuous representation for use in controlling car.
* See code in [discrete\_trajectory\_planner.h](https://github.com/ericlavigne/CarND-Path-Planning/blob/master/src/discrete_trajectory_planner.h) and [discrete\_trajectory\_planner.cpp](https://github.com/ericlavigne/CarND-Path-Planning/blob/master/src/discrete_trajectory_planner.cpp).

#### Frenet Conversion

* Converts between cartesian (x,y) and frenet (s,d) coordinates.
* Needed because trajectory planning uses frenet while sensors and controller use cartesian.
* Uses splines for much better accuracy compared with linear interpolation.
* See code in [track.h](https://github.com/ericlavigne/CarND-Path-Planning/blob/master/src/track.h) and [track.cpp](https://github.com/ericlavigne/CarND-Path-Planning/blob/master/src/track.cpp).

#### Controller

* Directly controls speed and path of car to follow waypoints from Trajectory Planner.
* Determines acceleration based on assumption of constant acceleration between current state and one second later in the Trajectory Planner's waypoints.
* Determines precise path in 0.02 second increments by applying spline from current state through all of the Trajectory Plannner's waypoints.
* See code in [controller.h](https://github.com/ericlavigne/CarND-Path-Planning/blob/master/src/controller.h) and [controller.cpp](https://github.com/ericlavigne/CarND-Path-Planning/blob/master/src/controller.cpp).

### Usage

#### Downloading the Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

#### Installing Dependencies

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

#### Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`

#### Running Automated Tests

`./test_path_planning`

#### Running Planner with the Simulator

1. Run the planner: `./path_planning`
2. Run the simulator.

### Background

#### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### Highway Map

The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

#### Data from Simulator

Here is the data provided from the Simulator to the C++ Program

##### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

##### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

##### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

##### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

#### Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

