# How to Run

## Six Wheeled Platform

#### Connecting to the Jetson PC

Connect to the *CARS* wifi with password *Test1234*
Then, ssh terminals to Jetson PC:  

`ssh jetson3@192.168.1.66`
password for the jetson2: *jetsongtarc*

#### Running the project

System is compromised of four basic components.

- Motor Controller Interface which takes cmd_vel message and translates it into motor commands.
- Odometry agent which initializes sensors and starts slam algorithms. Also publishes necessary transform trees.
- Local planner which calculates cmd_vel given odometry and path data
- Global planner which calculates the path from start to the end.

To run the motor controller:
Please First open the metal switch behind the car. You will see start up led blinking whan motor controller first starts. Once the motor controler start one of each Red Yellow and Red Leds will be lighten up.
After that run the folowing command in the jetson.

`rosrun serial_6w serial_communicator_6w`
**TODO: Write the agent names comes with the roslaunch (and their brief descriptions)**

After This command serial communication between motor controler and jeson will be started. To check the porcess you can check that only green leds over the motor controller will be lighting up.

The node's name is  serial_communicator it publishes /serial_communicator/motor_controler_info  and listens  /serial_communicator/motor_commands topics

Next step is to start the low level speed and steering controller. These nodes takes cmd_vel and translates it into correct messages for the Motor Controller.

To start linear controller:

`rosrun low_level_speed_controller six_wheel_low_level_controller`
**TODO: Write the agent names comes with the roslaunch (and their brief descriptions)**

This starts the controller node with parameters in param folder. By default this node listens to /cmd_vel and publishes commands with 50.0Hz and uses 4% duty cycle for takeoff. Topic names can be changed from the yaml file and control type, timeout and publish rate can be changed inside the node. Example setup can be found in nodes source code.


To start the steering controller:

`rosrun low_level_steering_controller six_wheel_low_level_steering_controller`
**TODO: Write the agent names comes with the roslaunch (and their brief descriptions)**

This is only an interface which takes rad/s and translates it into servo command. Details can be found in the  architecture part.


To start odometry and sensors:

`roslaunch odometry_agent odometry_agent.launch`
**TODO: Write the agent names comes with the roslaunch (and their brief descriptions)**

This package launches hector_slam, LIDAR driver and camera driver. All data from sensors are fused using robot_localization package. Imu data is retrieved from realsense camera.



To start all planners and pid controllers for them:

`roslaunch pose_follower diff_drive.launch`
**TODO: Write the agent names comes with the roslaunch (and their brief descriptions)**

In cfg folder of pose_follower package also there is a rviz config for visualization. Pid parameters can be changed from pid_controller.launch file in pose_follower package. This launch file basically sets up move_base system with parameters and loads controllers.

To also start PID Controler
`roslaunch pose_follower six_pid_controller.launch`
**TODO: Write the agent names comes with the roslaunch (and their brief descriptions)**
**TODO: how to run RC**

If you run in the Jetson PC directly (with a monitor), run rviz to control the car and its navigation:
`rosrun rviz rviz`
**TODO: here write how to add the config file**
 - WRITE Data that are available already with the config
 - How to give a goal (mention about the final orientation) -->
 - In cfg folder of pose_follower package also there is a rviz config for visualization. Pid parameters can be changed from pid_controller.launch file in pose_follower package. This launch file basically sets up move_base system with parameters and loads controllers.

##### Emergency Stop
**TODO: which terminals to kill, or additional commands to stop (manual methods)**

#### Interfacing
** TODO Services, topics their descriptions (high-level)
**TODO: global planner practical parameters (the ones that launch file imports). For all the rest provide the online links**
**TODO: local planner practical parameters, e.g. how to tune PID parameters, reaching goal point etc.**
**TODO: local cost map parameters (from move base)**
**TODO: how to send path arrays (overwriting the global planner)**
**TODO: odometry: Localization does fusion (how to change the weights of odometries?) and how to select the sensors/types/values from the launch file **
