# TODOs for week3

## Robot Start

After connecting to the TurtleBot, in you computer's command terminal run
```bash
$ ssh ubuntu@[IP]
```
where `[IP]` is the IP address of the TB. The password for logging in is `turtlebot`.
Once logged in, in the TB's command shell run:
```bash
$ ros2 launch turtlebot3_bringup robot.launch.py
```

## Exercise 1

1.	Get the the main [file](week3/robot_control_exercise.m) and the [file](week3/TurtleBotVisualise.m) to visualise the robot for reading robot sensor data.
2.	Modify the script to control the robot:
    - implement PID to control the robot's heading on [line 53](week3/robot_control_exercise.m?ref_type=heads#L53);
    - implement PID to control the robot's position on [line 56](week3/robot_control_exercise.m?ref_type=heads#L56).

## Common issues

If you apply too high an angular and linear speed to the robot, it might start going in the wrong direction, as there is a speed safety control on the robot for each of the wheels.

Clip the velocity to 0.1 instead of 0.2, and the problem should be fixed.

## Finish

Once you are done working with the TB, shut it down propoerly, by running
```bash
$ sudo shutdown now
```