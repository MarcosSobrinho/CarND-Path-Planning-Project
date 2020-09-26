# Path Planning Submission

## Code Structure
The code is based on the [Q&A video](https://www.youtube.com/watch?time_continue=3614&v=7sI3VHFPP0w&feature=emb_logo) for this project. It is dvided into the following sections:
* Definition of constant values
* Sensor fusion evaluation
* Lane change consideration
* Spline starting points
* Spline end points
* Trajectory generation

### Definition of constant values and other variables
* 50 points in trajectory
* 0.02 seconds between trajectory points
* 22.3 m/s maximum speed
* 9.5 m/s² maximum acceleration

These values lead to further definitions:
* max distance between trajectory points = 0.02 * 22.3
* max speed change in cycle = 9.5 * 0.02

Furthermore there is a `lane` integer, that indicates which lane the ego vehicle is in. 0 being the left lane, 1 the middle lane and 2 the right lane.

### Sensor fusion evaluation
The goal of this part is to update the `too_close` array of bools. It consists of 3 cells. When  `too_close[lane]` of is true, it means, that a lane change should be considered. When `too_close[lane-1]` is true, the vehicle cannot change to the left. The same rule holds for `too_close[lane+1]`. 

The modification of `too_close` is done by the `EvaluateFusionData` function. It reads the sensor fusion data and does the following for all vehicles in sensor fusion: 
* If another vehicle is in the same lane as the ego vehicle and in a facing forward distance up to 30m (Frenet), then `too_close[lane]` is set to true. 
* If the other vehicle is left of the ego vehicle lane within a lane width and in a forward facing distance between -5m and 30m, then `too_close[lane-1]` is set to true.
* If the other vehicle is right of the ego vehicle lane within a lane width and in a forward facing distance between -5m and 30m, then `too_close[lane+1]` is set to true.

The -5m is a longitudinal buffer distance. Without it there would be the possiblity of the ego vehicle changing lanes, even though another car is driving next to it in the target lane. With the buffer distance, lane changing is only done when the ego vehicle is at least 5m or 30m behind a car in the target lane. 

### Lane change consideration
The function `ConsiderLaneChange` takes `too_close` and `lane` as parameters, to decide, if a lane change shall be done or not. It is called if `too_close[lane]` is true. If it is not in the middle lane, it simply checks `too_close` of the target lane. If it is in the middle lane, it first checks the left lane and then the right lane. The left lane is checked first, because the ego vehicle is moving on a circuit. The left lane is the inner most lane and thus should be the fastest to close the loop.
`ConsiderLaneChange` returns the `lane` the vehicle shall target.

### Spline starting points
In case a previous trajectory does not exist, the vehicle's current position has to be one point. Another point that makes sense, is an arbitrary point in negative vehicle heading direction. This guarantees, that the spline goes through the vehicle in direction of the yaw angle. 
If however a previous trajectory exists, the last two points are taken as reference. This allows to append new spline points to what is left of the last cycles trajectory, thus leading to less computational effort.

### Spline end points
The end points are simply points on the targeted `lane`. They are set to be further away than the trajectorie's end point to guarantee smoothness.

### Trajectory generation
Since the spline only gives a path, this final section deals with spacing the points on the spline in a way that reduces jerk. This is reached through a linear approximation of the spline. The linear approximation is then divided in parts of equal length. The length is determined through the vehicles reference velocity. To facilitate this computation, the coordinates are first transformed to vehicle coordinates and after the compotation back to global coordinates. The object `transform` of type `CoordinateTransform` handles the transformations. 
Finally, points are added to the leftover trajectory from the last cycle until the trajectory has 50 points again. That trajectory is then sent to the simulator.

## Conclusion
This simple path planner fulfills the [acceptance criteria](https://review.udacity.com/#!/rubrics/1971/view) most of the time. There are cases, when another vehicle suddenly changes lanes and runs right into the planned trajectory. In these cases, the planner fails, because it is using previous points. A better trajectory planner might handle this more elegant, but I think that it would probably have to violate accelleration and jerk limits to avoid a collision. 
I have tested the planner for about three hours and saw the incident described above twice. I think it happens quite rarely and is not a faulty behavior of a planner, but of the other cars driver ;) 