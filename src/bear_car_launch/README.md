# bear_car_launch

This package collects various system relevant launch files, configuration files and static maps for 4WD platform.

## Main launch files 

Choice between
	- cartographer_slam
	- hector_slam
	- static maps (All static maps .pgm and .yaml files can be found in `static_maps`)

Choice between
	- legacy low level controller 
	- New ROS Control framework based low level controllers 
	
* autonomous_navigation_fourwd 
	- For real car
* autonomous_navigation_fourwd_sim
	- For real car
* odom_sensor_start
	- Launches odometry_agent
	- If simulation = True,
		- Launches simulation_transforms static transform publisher
		- Launches steer_force converter to convert /ackermann_cmd OR /cmd_vel (depends on low level controller used)--> steer_force & steer_angle
	- If Simulation = False,
		- Launches fourwd_transforms static transform publisher
* Navigation_SLAM 
	- For launching movebase/locomotor based navigation stack & choice of SLAM 

## Folders contatining secondary Launch files

Main Launch files is as follows:
### hardware
- Launches files for all sensors on the real car

### transforms
- Switches the static_transform publishers based on whether the setup is simulation or real car

### cartographer_slam
- Loads "cfg/cartographer_slam/bear_car.lua" configuaration file
- Launches cartographer_node
- Launches image_pipeline to process camera depth input to PointCloud2

### hector_slam
- Launches HECTOR SLAM if enabled with the required parameters (no seperate cfg file)

### move_base
- Entire set up of navigation stack
   - Loads costmap .yaml files from `cfg/move_base`
   - Loads the global and local planners. Their assosiated .yaml configuartion is fetched from  `cfg/move_base`
   - choice of multiple global and local planners other than the default move_base planners
   		- Global Planners
   			- sbpl planner / default planner (only default planner is properly working)
   		- Local Planner
   			- Teb Planner (needs further tuning on realcar, working fine in simulation)
   			- MPC (tested in both real car and simulation)
   			- e_band Planner (untested)
   			- RS_band planner (untested)
   			- DWA_Planner (untested)
###  EKF_robot_localization
	- Launches robot_localization Odom fusion node with the cfg parameters in the `cfg/EKF_robot_localization/`

## Cfg folder consists of all YAML/cgf files needed by the packages (follows same structure as launch files folder)

## Other Packages

### Remote-controlled (RC) driving ( has not setup)

Depending on the used platform use either

`roslaunch bear_car_launch direct_rc_fourwd.launch` 

or

`roslaunch bear_car_launch direct_rc_sixwd.launch`

### morse_telop (has not setup)

Running nodes for tele-operation of simulated morse robots.

Nodes are publishing twist messages to the topic `/robot/basic_actuate`

Keyboard control:
`roslaunch bear_car_launch morse_teleop.launch use_keyboard:=true` 

Gamepad control:
`roslaunch bear_car_launch morse_teleop.launch use_keyboard:=false`

Without the parameter `use_keyboard` default will be keyboard.

### Morse RC teleoperation (has not setup)

You can also plug the RC to your machine and run following launch file for controlling
either 6WD or 4WD platform

`roslaunch ackermann_rc rc_driver_morse.launch` 
