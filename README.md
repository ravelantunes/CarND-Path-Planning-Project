# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Goals
The goal of this project was to build a path planning algorithm that takes into account traffic conditions, physics and
jerk constriains, and a goal destination to generate the most optimal path.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program


## Details

1. The car uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

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
    
    
## Ego Car Finite States

The path planning algorithm uses a finite state machine with 3 states: KEEP_LANE, PREPARE_LANE_CHANGE, and CHANGE_LANE.
Initial state is KEEP_LANE.

KEEP_LANE state: Ego vehicle tries to cruise near the speed limit (50mph). If there's another vehicle in front of the
ego vehicle on the same lane less than 50 meters away, it will gradually reduces vehicle speed to match the car in front
of it and change state to PREPARE_LANE_CHANGE.

PREPARE_LANE_CHANGE state: Ego car asses the most appropriate lane to be at. It then tests if path is feasible by
considering other vehicles on the road and vehicle dynamic constraints. If path is not feasible, stays in
PREPARE_LANE_CHANGE state until a better path is found. Once a better path is found, switch state to CHANGE_LANE.

On CHANGE_LANE vehicle will start changing lane to the previously determine best lane. Once the center of the lane is achieved,
state will change back to KEEP_LANE.


## Trajectory Generation

To make smooth lane changes and accelerations, the car tries to minimize jerk on path planning (jerk is the third
derivative of position). To do that, Jerk Minimizing Trajectory is used. Before the trajectory is generated, the path in
x,y coordinates are first converted into Frenet coordinates, so a polynomial can easily fitted.

Once a polynomial is fitted, the path is generated on Frenet space, and converted to vehicle local x,y coordinates.
A spline is then fitted to smooth the trajectory between points fitted on the polynomial, and the results of the spline
points are then converted into the global x,y coordinates.