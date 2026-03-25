Requirements: Corke's robotics toolbox and Image processing toolbox needs to be installed.

This week you will:

- Create and modify your own binary occupancy grid map over an area in Shannon containg a corner. Follow the examples in Corke Chapter 5.4, Excurse 5.7 on how to do this. Also have a look at the example code in ```PRM_example_CORKE```. Note, remove the ```...``` from the command, i.e. ```map.setOccupancy([40 20], ...  true(10,60), "grid");``` to ```map.setOccupancy([40 20], true(10,60), "grid");```.
- Test different parameters for PRM. Generate a path from a start location to an end location of choice. Opt for a path that goes around a corner, rather than just a straight path.
- Have the Turtlebot track the path. Complete the ```createMap.m```. Thereafter, use that and the ```createPath.m``` in your robot control program. Hints: Fix the initial position of the Turtlebot with respect to your map. Generate a trajectory from the path (see week 3 slides).



