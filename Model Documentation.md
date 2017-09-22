### General

To complete the task, I have chosen the same approach as in [Q&A video](https://www.youtube.com/watch?v=3QP3hJHm4WM) for the project. We control car's speed by increasing\decreasing it no more than 0.224 mph. It allows us not to worry about jerks. I use [provided methods](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/map.cpp#L28) for conversion from frenet to cartesian coordinates and vice versa.

To control car's behavior I use [final states machine](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/FSM.cpp) with [six possible states](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/estimator.h#L11):
* _CS_ (constant speed) - initial car's state. It is never actually used.
* _KL_ (keep line) - the car should stay in the same lane. [Depending on the situation](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/FSM.cpp#L155) we can increase car's speed (no cars ahead), adjust speed to the car's in front of us or just drive with maximum speed.
* _PLCL_ (prepare line change left) - the car [prepares to change the line](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/FSM.cpp#L228). The main idea that we increase our speed disregard whether we have a car in front of us or not.
* _PLCR_ (prepare line change right) - the same as _PLCL_ but for the right turn.
* _LCL_ (line change left) - we try to adjust our speed in [a new lane](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/FSM.cpp#L214). The idea is the same as for _KL_. The only difference is that we change `lane` value.
* _LCR_ (line change right) - the same as _LCL_ but for the right turn.

Initially, we get coordinates in global map and frenet car's coordinates. To predict and control car's behavior we [estimate vehicle's position](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/vehicle.cpp#L99) using global coordinates. Then we [get frenet](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/vehicle.cpp#L116) coordinates and use them to make the best decision using cost function.

When we have chosen the best future state, we [build new waypoints](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/pathplanner.cpp#L78) for our car. To avoid jerks, we use waypoints from our [previously formed path](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/trajectory.cpp#L54). To build the rest of the way we use [frenet coordinates](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/trajectory.cpp#L137) and based on them get global coordinates.

To avoid jerks, we have to build our waypoints by dividing our path with the same time frames. For this purpose, we convert our global coordinates [to local one](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/trajectory.cpp#L42) and use spline library to build a smooth path. Then we build [the path up to desired distance](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/trajectory.cpp#L70). The last step is to convert local coordinates [back to global](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/trajectory.cpp#L78).

### Cost functions

To choose the best state in FSM we use [five cost functions](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/estimator.h#L75):
* [`change_lane_cost`](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/estimator.cpp#L19) - we use to prefer staying in the lane than switching the lanes.
* [`inefficiency_cost`](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/estimator.cpp#L28) - if we can drive in another lane with better speed, we should change the lane
* [`free_line_cost`](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/estimator.cpp#L52) - the same purpose as above. We want to force the car to change the lane if we do not see any obstacles in another lane.
* [`collision_cost`](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/estimator.cpp#L38) - if we found a collision in another lane, we must not change the lane (`COLLISION` has the largest weight)
* [`buffer_cost`](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/estimator.cpp#L66) - we use it to switch between prepare lane change and actual lane change. If we see some car in front of use, we should change the lane.

### Predictions

Every time frame we update information about cars around us and [generate predictions](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/pathplanner.cpp#L52) based on their metrics.
Then we [choose plausible future states](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/FSM.cpp#L52) for our current state.
Next step is to [calculate cost](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/FSM.cpp#L80) for every state and choose the cheapest one. For this purpose, we have to [generate car's trajectory](https://github.com/Helen1987/CarND-Path-Planning-Project/blob/master/src/FSM.cpp#L99) based on predictions.

### Points to improve

Conversion methods have quite poor accuracy. As a result, I still can sometimes see odd car's behavior and accidents caused by this reason. 
