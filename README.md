# tuw_teleop
Simple nodes to teleoperate vehicles. 

# tuw_gamepad: 
Nodes optimized for a Logitech F710 Gamepad 
- roslaunch tuw_gamepad gamepad_twist.launch 

# tuw_joy2twist: 
Works with any Joystick/Gamepad supported by ros-joy / joy_node
Configure different controllers (button and axis assignment) with yaml files
in the cfg directory.
Depending on parameter 'publisher_type' it publishes:
- case 0: geometry_msgs/Twist
- case 1: tuw_nav_msgs/JointsIWS using 1+1 (1 steering, 1 revolute)

- roslaunch tuw_joy2twist teleop_joy2twist.launch gamepad:=logitech_f510 publisher_type:=0

# tuw_keyboard2twist: 
Keyboard teleop node
- roslaunch tuw_keyboard2twist teleop_keyboard_twist.launch 
<<<<<<< HEAD

# tuw_gui2iws: (removed)
graphical interface to control an independent steering platform.
- roslaunch tuw_gui2iws teleop_gui_iws.launch 
=======

