# CarND-Path-Planning-Project
Path planning for highway driving.
   

### Goals
#### Rubric points

1. The code compiles correctly - code must compile without errors with cmake and make.
2. The car is able to drive at least 4.32 miles without incident.
3. The car drives according to the speed limit.
4. Max Acceleration and Jerk are not exceeded.
5. Car does not have collisions.
6. The car stays in its lane, except for the time between changing lanes.
7. The car is able to change lanes.
8. There is a reflection on how to generate paths - the code model for generating paths is described in detail.

In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Input data: localization and sensor fusion data, sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.
 

## Code Description

The code is organized as follows: the main.cpp file implements the behavior planning logic and path planning, while planner.cpp implements scene understanding based on the inputs from the sensor data fusion. Within the planner.cpp, following concepts are implemented:
1. For a given lane, front objects from the sensor data fusion are assigned to this lane (planner.cpp lines 23 to 34. After this, the closest vehicle is selected a target and returned as a Lane class member. It can then be accessed within the main.cpp file by calling the members of the corresponding class instance (egoLane, neighb1Lane or neighb2Lane). 
2. A similar logic for extraction of the closest rear vehicle is implemented through the member function Lane::rear_traffic_check (from line 65).

For the current ego lane, our vehicle tracks the front target vehicle and, if the latter drives so slow that the safe distance to the ego vehicle is violated, checks for the occupancy of the neighboring lanes while initiating own deceleration (main.cpp lines 204 to 240). If any particular lane is free, that means, the distance to the potential target and rear vehicle in this lane is safe, ego vehicle will initiate a lane change by manipulating the value of the *lane* variable (setting it to the value of the target lane). This value will be passed into the path planing section.

If the target vehicle is far enough, ego vehicle starts acceleration in order to achieve the maximum allowed speed.

Additionally, a check for cut-ins is implemented (main.cpp lines 167 to 199). As vehicles from neighboring lanes might also change lanes (and sometimes do so in quite an aggressive manner), the ego vehicle should foresee such situation and react by applying deceleration in order to avoid the collision with the cut-in vehicle. The check implementation is rule-based - a possible improvement would be an advanced tracker or a learned model based on a larger data set (e.g., lateral speed and position for the vehicles on the neighboring lanes vs. labels like "0" for "no cut-in" or "1" for "cut-in").

The trajectory planning part is implemented in main.cpp in lines 264 to 352.
The algorithm uses prediction of the trajectory points based on the path from the earlier cycle (starting from line 277 in main.cpp). Furthermore, we define anchor points based on the map data (map_waypoints, see lines 293-295) for a lookahead path coordinate extraction. Please note that lines 293-295 also consist of the interface to the lane change (variable *lane* is propagated to the function *getXY* and offsets the trajectory to the neighboring lane if the value of the *lane* variable changes and thus assumes a lane change). We also rotate the resulting coordinates based on the vehicle orientation in the current frame. We employ the spline class in order to introduce a smoothly interpolated representation of the trajectory points (line 316). Spacing of the trajectory points is defined based on the current set velocity (line 335). The X-coordinate is then defined by an iterative increase of the spacing compared to the previous point, the Y-coordinate is found with the help of the *spline* class by using the X-coordinates as input (line 337). After rotating the trajectory points to the original coordinate system, we pass the calculated trajectory to the communication socket (lines 354-355).

Further improvements can be implemeted:
* longitudinal control - set speed can be set into relation to the speed of the target vehicle
* strategic lane change - defined by analyzing the average speed in the candidate lanes and deciding to take on the lane that will minimize the delay
* aggressive driving - accelerating and decelerating while violating the defined safety distance in order to enable an aggressive lane change (in order to switch to the neighboring lane if the traffic there is faster than in the ego lane) even by violating the defined safe distances to the front and rear vehicles. Here safety mechanisms shall still be taken into account, e.g. by calculating the minimum viable safety distance based on the velocities of the front and rear vehicles and taken into account the reaction capabilities of the ego vehicle.
