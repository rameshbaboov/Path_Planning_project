# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

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

# Algorithm

The project uses the below algorithm:
   1. Read the current position of Car and its Yaw (x,y,s,d,yaw) from the JSON file provided by simulator
   2. Read the sensor fusion data of the nearby cars to track their position
   3. For each of the identified vehicle do the following:
         a. Check its position to determine if these cars on the same side as the ego car
         b. If these cars are either on the right side or left side of the ego car lane and within distance of 30ms either ahead             or behind.
   4. If there is any car in the same lane that is within 30ms, then do the following
         a. If the car is not in the fastest lane (lane 0), then move to the next fast lane on the left side, provided there are             no cars on that lane either ahead or behind within 30ms
         b. If the car is not in the slow lane (lane 2), then move to the next slow lane, as the vehicle is getting choked in                traffic, provided there are no cars on that lane either ahead or behind
         c. If neither of two are not possible and lane cannot be changed, then slow down the vehicle and increase the speed                 only if the vehicles disappear from vicinity of 30ms
5. Safety algorithms - Following algorithms ensure safety of car and avoid any collision
         a. Predict the position of the other cars and not the current position.
         b. Reduce the speed of the ego car if lane change fails. But use the relative velocity times a factor of 0.0325 to                 reduce the speed
         c. This ensures that if the speed is not randomly reduced but based on the relative
            velocity.
         d. Absolute value is used in relative velocity to ensure that if any vehicle jumps in
            front of the ego vehicle, no collision happens
6. Algorithm to ensure unnecessary slowdown of vehicle:
         a. Whenever lane change is not possible and vehicle slows down, a lane change is
            checked after the vehicle moves out of vicinity of 30ms.
         b. This ensure that the vehicle do not unnecessarily tailgate the vehicle that is
            blocking the path and vehicle moves to next available lane and overtakes the car
         
7. Waypoint generation - The way point are generated based on below logic
         a. If the vehicle is in the start, then current position of car and previous position
            based on Yaw is calculated
         b. If the vehicle is already moving, last two previous positions are retrieved and its
            yaw is calculated
         c. Next three way points are calculated with a distance of 30ms. getXY function is
            used as Frenet co-ordinates are used to ensure that car can easily nagivate while
            staying in the lane
         d. All the five points are transformed to ensure that car moves in the x direction
         e. Spline is used for generating the remaining points that were not consumed by
            the car and hence a trajectory of 50 points are generated
         f. Target Y is calculated using the spline for a distance of 30ms and the 50 points
            are evenly distributed
         g. The points are rotated and pushed to controller so that trajectory is generated
8. Control of vehicle
         a. Staying in lane – Frenet coordinate is used to drive the car in required lane
         b. Controlling of speed – Speed is controlled by spacing the points generated by
         spline.
         c. Lane change. Lane change is managed by a variable Lane that is calculated as
         follows

      ## LANE           Lane description        Position
         0              Fast Lane               0 to 4
         1              Middle lane             4 to 8
         2              Slow lane               8 to 12

         d. The formula 2+4*lane is used to drive the car in center of lane
         
# Rubric

The code compiles correctly – Code compiles correctly without any error
   * The car is able to drive at least 4.32 miles without incident. – Yes the car is able to drive
     without any incident
   * The car drives according to the speed limit. – Yes. Maximum speed limit is 49.5mph
   * Max Acceleration and Jerk are not Exceeded – Within limits
   * Car does not have collisions – Algorithm ensure that vehicle is slow down if there are any
     changes of collision and also vehicle does not change lane if there are vehicles within 30
     meters either ahead or behind
   * The car stays in its lane, except for the time between changing lanes – The lane change
      happens smoothly
   * The car is able to change lanes – The car changes lane if there are any cars that are blocking
     the way
   * There is a reflection on how to generate paths – Yes. The Reflection is easily seen
https://github.com/rameshbaboov/Path_Planning_project/blob/master/img/1.JPG

# Acknowledgement

Thanks to Udacity team for providing FAQ Video with starter code. It was very helpful
