# How to Run

## Four Wheeled (4WD) Platform

### Connecting to the Jetson PC

Connect to the *CARS* wifi with password *Test1234*
Then, ssh terminals to Jetson PC:  

`ssh jetson2@192.168.1.188`
password for the jetson2: *jetsongtarc*

### Running the project

#### Running Autonomous Navigation

System is compromised of four basic components.

- Motor Controller Interface which takes cmd_vel message and translates it into motor commands.
- Odometry agent which initializes sensors and starts slam algorithms. Also publishes necessary transform trees.
- Local planner which calculates cmd_vel given odometry and path data
- Global planner which calculates the path from start to the end.

You can run the entire autonomous navigation stack with

`roslaunch bear_car_launch fourwd_autonomous_navigation_fourwd.launch`

#### Running Autonomous Navigation Step-by-Step

If you only want to follow the process manually, please follow commands below.

To run sensor nodes separately:

`roslaunch bear_car_launch sensors.launch`
However, sensors are automatically started by the odometry agent below.

To run the motor controller:

`roslaunch vesc_driver vesc_driver_node.launch`

**TODO: Write the agent names comes with the roslaunch (and their brief descriptions)**

This starts ROS node for communicating with servo and BLDC.  Node launched is the vesc_driver_node. This agent handles all serial communication to VESC. It sends commands and periodically polls sensor data from VESC.

Next step is to start the low level speed and steering controller. These nodes take cmd_vel and translates it into correct messages for vesc_driver_node.



To start linear controller together with PID controller node:

`roslaunch low_level_speed_controller allg.launch is_four_wd:=true`

If PID controller is started separately (e.g. if you use navigation_stack.launch) you have to use

`roslaunch low_level_speed_controller four_wd.launch`

**TODO: Write the agent names comes with the roslaunch (and their brief descriptions)**

This starts the controller node with parameters in param folder. Controller is responsible for taking cmd_vel and generating necessary commands for VESC driver. This node also handles takeoff behaviour.



To start the steering controller:

`rosrun low_level_steering_controller low_level_steering_controller`

This is only an interface which takes rad/s and translates it into servo commands. Details can be found in the  architecture part.



To start odometry and sensors together:

`roslaunch odometry_agent odometry_agent.launch is_four_wd:=true`

**TODO: Write the agent names comes with the roslaunch (and their brief descriptions)**

This package launches hector_slam, LIDAR driver rplidar_ros and realsense camera driver. All data from sensors are fused using robot_localization package (an EKF is launched for the operation). IMU data is retrieved from realsense camera. Currently laser odometry and wheel odometry is used. IMU was causing oscillations in the pose which the problem may be solved by changing weights.


To start all planners and pid controllers for them:

`roslaunch pose_follower navigation_stack_fourwd.launch`

This will make the car run in autonomous mode.

#### Visualization:

If you run in the Jetson PC directly (with a monitor), run rviz to control the car and its navigation:
`rosrun rviz rviz`

Also With the configuration:

```rosrun rviz rviz -d `rospack find pose_follower`/rviz/rviz_navigation.rviz```

This is also wrapped within

`roslaunch pose_follower navigation_stack_rviz.launch`

### Running 4WD Simulation

A simulation environment is prepared for the 4WD car in MORSE environment. Make sure that you have latest MORSE master installed.
To run, first we need to import the MORSE project, then simply run the builder script:

`cd <project_folder>/morse`
`morse import fourwd`
`morse run fourwd fourwd/default.py`

This will initialize the environment. Note that you need relevant ROS nodes or projects running that interfaces the MORSE environment. If you would like to navigate in the environment, at least start a *roscore* so the interface between MORSE and ROS starts.

For the odometry agent, this time we run with the parameter that is stating it is simulation

`roslaunch odometry_agent odometry_agent.launch is_four_wd:=true is_simulation:=true`

####RC mode for running the project and to control the simulation

To start the **RC mode**, there are two options currently implemented.  

First option is to use rc_driver_vesc launch file in ackermann_rc package. This launch file launches a node that directly interfaces with vesc_driver. Hence using this launch file with low_level_speed and steer controllers is not recommended. 

Second option is to use the interface created for morse. This is included in rc_driver_morse launch file. Morse accepts twist message for control and that twist output can be fed into low level controllers, which would result in a better control. 

Third, currently not fully implemented but possible solution is to use rc_driver launch file. This node outputs AckermannDrive msg and linear low speed controller can handle this message. But steering controller lacks a simple listener to this message type.


To run in RC mode, *DO NOT launch pose_follower navigation_stack.launch*. To run in RC mode with our low-level controller (integrated, but issues remain. See the open issues page), assign `true` to `use_twist` parameter under *ackermann_rc/param/rc_driver_vesc.yaml*, then run the rc_driver_vesc.launch below:

`roslaunch ackermann_rc rc_driver_vesc.launch`


To run using morse interface, RC interface is also created. However, for the ease of use, we strongly recommend developing a teleop packages to be able to control the car with the keyboard.

`roslaunch ackermann_rc rc_driver_morse.launch`

None of these launch files require odometry_agent to be running.

You can also directly run the entire setup with just one launch file for direct rc control with

`roslaunch bear_car_launch direct_rc_fourwd.launch`

### Tuning the control parameters

For the purpose, a YAML file is created to interface the low level control units. This directly loads the parameters to the VESC driver.
**TODO: where is this YAML file??? ackermann_rc/param/rc_driver_vesc.yaml**


**TODO: here write how to add the config file**

 - WRITE Data that are available already with the config
 - How to give a goal (mention about the final orientation) -->
 - In cfg folder of pose_follower package also there is a rviz config for visualization. Pid parameters can be changed from pid_controller.launch file in pose_follower package. This launch file basically sets up move_base system with parameters and loads controllers.

### Emergency Stop

For emergency stop, there are couple of methods. Killing the terminal with **low_level_speed_controller** shuts down communication with VESC. This causes VESC to stop in a short while usually under a second.

If **navigation_stack** is killed, cmd_vel generation is stopped. This causes low_level_speed_controller to stop the car after command_timeout time is passed. This parameter is changeable from corresponding yaml file of low_level_speed_controller.

Currently, there is only one hardware kill switch on the battery that supplies digital boards. Closing it would shutdown motors immediately.


### Interfacing

#### VESC Driver:

​	/sensors/core: This topic outputs the internal values of VESC like erpm, input voltage, current, temp...

/commands/motor/... : There are several topics like this. They receive double values to corresponding interfaces. For example a 0.04 published to /commands/motor/duty_cycle causes a switch to duty_cycle control with 4% duty_cycle.

#### Low Level Controllers

​	/cmd_vel: This basically the only input topic they need. They also listen to the odometry data. Topic names can be changed from the yaml file four_wd_params.yaml. In that file you can also configure takeoff duty_cycle levels.

#### Local And Global Planners

They generate the cmd_vel necessary to complete the path. All PID controller parameters can be changed from`pose_follower/launch/pid_controller.launch`.  Also in cfg/carlike folder, all parameters necessary for the system is present. In pose_follower.yaml you can change goal point tolerance and goal point timeout as well as controller limits.  To send a goal, you can use rviz 2D nav goal interface. Also move_base also has an action interface to send goal points.


* TODO Services, topics their descriptions (high-level)
* TODO: global planner practical parameters (the ones that launch file imports). For all the rest provide the online links**
* TODO: local planner practical parameters, e.g. how to tune PID parameters, reaching goal point etc.**
* TODO: local cost map parameters (from move base)**
* TODO: how to send path arrays (overwriting the global planner)**
* TODO: odometry: Localization does fusion (how to change the weights of odometries?) and how to select the sensors/types/values from the launch file **
