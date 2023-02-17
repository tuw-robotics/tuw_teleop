# tuw_teleop
Simple nodes to teleoperate vehicles. 

## tuw_teleop
Metapackage.

## tuw_gamepad: 
Node optimized for the Logitech F510 and Logitech F710 Gamepad to publish [`geometry_msgs/Twist`][geometry_msgs/Twist] and [`tuw_msgs/JointsIWS`][tuw_msgs/JointsIWS].

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

## tuw_keyboard:
Node to publish commands based on keyboard input.

## tuw_patroling:
Node to publish predefined goals for the robot.
