# How to Run

## Four Wheeled Platform

System is compromised of four basic components.

- Motor Controller Interface which takes cmd_vel message and translates it into motor commands.
- Odometry agent which initializes sensors and starts slam algorithms. Also publishes necessary transform trees.
- Local planner which calculates cmd_vel given odometry and path data
- Global planner which calculates the path from start to the end.



To run the motor controller:

`roslaunch vesc_driver vesc_driver_node.launch`

This starts ROS node for communicating with servo and bldc. /commands/servo/position writes to servo and /commands/motor/* topics are inputs to corresponding interfaces.



Next step is to start the low level speed and steering controller. These nodes takes cmd_vel and translates it into correct messages for vesc_driver_node.



To start linear controller:

`roslaunch low_level_speed_controller allg.launch is_four_wd:=true` 

This starts the controller node with parameters in param folder. By default this node listens to /cmd_vel and publishes commands with 50.0Hz and uses 4% duty cycle for takeoff. Topic names can be changed from the yaml file and control type, timeout and publish rate can be changed inside the node. Example setup can be found in nodes source code.



To start the steering controller:

`rosrun low_level_steering_controller low_level_steer_controller`

This is only an interface which takes rad/s and translates it into servo command. Details can be found in the  architecture part.



To start odometry and sensors: 

`roslaunch odometry_agent odometry_agent.launch`

This package launches hector_slam, LIDAR driver and camera driver. All data from sensors are fused using robot_localization package. Imu data is retrieved from realsense camera. 



To start all planners and pid controllers for them:

`roslaunch pose_follower navigation_stack.launch`

In cfg folder of pose_follower package also there is a rviz config for visualization. Pid parameters can be changed from pid_controller.launch file in pose_follower package. This launch file basically sets up move_base system with parameters and loads controllers. 

