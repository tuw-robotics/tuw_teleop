# tuw_patrolling

This node provides the means to make the robot travel between predefined goal poses which are in the **patroling**.yaml file.

* ```roslaunch tuw_patroling patroling.launch```

#### Adding new goal poses:

* Firstly, update the **number_of_goals** paramater to reflect the total number of goal poses.

* Second, copy one of the existing goal poses and change the letters accordingly. The goal pose names follow an **ascending** order so make sure not the give arbitrary letters; after A, B has to follow.

* Example format:

		goal_name_A: "A"
		goal_position_x_A: -1.13491773
		goal_position_y_A: -0.2857383
		goal_orientation_z_A: 0.99914570
		goal_orientation_w_A: -0.04132
		
		
#### Changing the waiting time at goal poses:

* Change the **wait_time** parameter in the **patroling** file which reflects the waiting time in seconds.
 
* The waiting time is determined by the frequency of _robot info_ callback. By default this is 1 Hz so if an unexpected wait time occours, checking the _robot info_ frequency is the first thing to do.
