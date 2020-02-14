# bear_car_launch

This package collects various system relevant launch files, configuration files and static maps for platfor 4WD.

## hardware
- Launches files for all sensors on the real car

## transforms
- Switches the static_transform publishers based on whether the setup is simulation or real car

## cartographer_slam launch
- Loads "cfg/cartographer_slam/bear_car.lua" configuaration file
- Launches cartographer_node
- Launches image_pipeline to process camera depth input to PointCloud2

## hector_slam
- Loads the "hector_configuaration" file as an input for hector_slam

## move_base
- Entire set up of navigation stack
   - Loads costmap .yaml files from `cfg/move_base`
   - Loads the global and local planners. Their assosiated .yaml configuartion is fetched from  `cfg/move_base`
   - choice of multiple global and local planners other than the default move_base planners
   		- Global Planners
   			- sbpl planner / defalut planner (only default planner is properly working)
   		- Local Planner
   			- Teb Planner (needs further tuning on realcar, working fine in simulation)
   			- MPC (tested in both real car and simulation)
   			- e_band Planner (untested)
   			- RS_band planner (untested)
   			- DWA_Planner (untested)

## odom_navigation_start
- Launches odometry_agent
	- If simulation = True,
		- Launches simulation_transforms static transform publisher
		- Launches steer_force converter to convert /cmd_vel --> steer_force & steer_angle
	- If Simulation = False,
		- Launches fourwd_transforms static transform publisher
- Choice of maps
	- cartographer_slam
	- hector_slam
	- static maps (All static maps .pgm and .yaml files can be found in `static_maps`)

To run full stack in simulation-
	- `source run_all`

## Autonomous Navigation (real car)
- Launches vesc and lowlevel controllers
- Launches odom_navigation_start with fourwd configuration

To run full stack on real car-
	- `roslaunch bear_car_launch autonomous_navigation_fourwd.launch`



## Remote-controlled (RC) driving ( has not setup)

Depending on the used platform use either

`roslaunch bear_car_launch direct_rc_fourwd.launch` 

or

`roslaunch bear_car_launch direct_rc_sixwd.launch`

## morse_telop (has not setup)

Running nodes for tele-operation of simulated morse robots.

Nodes are publishing twist messages to the topic `/robot/basic_actuate`

Keyboard control:
`roslaunch bear_car_launch morse_teleop.launch use_keyboard:=true` 

Gamepad control:
`roslaunch bear_car_launch morse_teleop.launch use_keyboard:=false`

Without the parameter `use_keyboard` default will be keyboard.

## Morse RC teleoperation (has not setup)

You can also plug the RC to your machine and run following launch file for controlling
either 6WD or 4WD platform

`roslaunch ackermann_rc rc_driver_morse.launch` 


## rplidar_a3.launch

Launch file for starting the lidar.

## low_level_starter.launch

The launch file to easily start six wheeled platform.

It is a launch file that starts the nodes:

- low_level_speed_controller/six_wheel_low_level_controller
- low_level_steering_controller/six_wheel_low_level_steering_controller
- serial_6w/serial_communicator_6w

## rs_imu

Launch file to start imu filter and sensor

rs_imu.launch is launched by odometry_agent. IMU filtered data is being fused under ekf for acceleration after filtering.








