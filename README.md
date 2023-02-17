# tuw_teleop
Simple nodes to teleoperate vehicles. 

## tuw_teleop
Metapackage.

## tuw_gamepad: 
Package containing a node to publish motion commands based on input on a gamepad. 
The node is optimized for the Logitech F510 and Logitech F710 Gamepad to publish [`geometry_msgs/Twist`][geometry_msgs/Twist] and [`tuw_msgs/JointsIWS`][tuw_msgs/JointsIWS] for various types of robots.

There are various launch files for the different configurations to operate this node.

To utilize the controller to steer a differential drive robot with `geometry_msgs/Twist` use:
```bash
  roslaunch tuw_gamepad twist_diffdrive.launch 
```

To utilize the controller to steer a differential drive robot with `tuw_msgs/JointsIWS` use:
```bash
  roslaunch tuw_gamepad iws_diffdrive.launch 
```

[geometry_msgs/Twist]: https://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/Twist.html
[tuw_msgs/JointsIWS]: https://github.com/tuw-robotics/tuw_msgs/blob/master/tuw_nav_msgs/msg/JointsIWS.msg

**Note:** The Logitech controller should be set to mode "X" (not "D") on the rear of the controller in order to behave properly.

## tuw_gamepad_python:
Package containing a node similar to `tuw_gamepad` but written in python with a mode modular approach.
Please check out the [`README.md`](./tuw_gamepad_python/README.md) in the package for more details.

**Note:** This node has very limited support for messages

## tuw_keyboard:
Node to publish commands based on keyboard input.

## tuw_patroling:
Node to publish predefined goals for the robot.
