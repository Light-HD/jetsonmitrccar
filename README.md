# Autonomous Racing using MIT RaceCar and Nvidia Jetson platform

## Abstract
The aim was to develop an autonomous race car based on the MIT racecar platform. The initial
task was to identify all the required components to achieve autonomous racing such as perception, planning and achieving 
loop closure. A preliminary design was made and iterated to include updated knowledge about the platform and changes in the
approach. 
Google Cartographer was selected as the primary SLAM algorithm because of its added benefits of integrating other sensor 
information for building the map. The navigation stack was based on the already available ROS move\textunderscore base. Various planners 
for global and local path planning were tried out. The final selecting ones were a simple A* as the main global planner 
and TEB with our own MPC package, as the two local planners, which have been used for performance comparison reasons. Many optimisation and parameterization was done for VESC BLDC motor controller. Finally, ROS controllers was integrated for low level control as it was an expandable and easily configurable for controling different actuators.

## Software Architecture 
![Alt text](Images/Software_arch.png?raw=true "Software Architecture")

## Results 
![Alt text](Images/sim_env.png?raw=true "Testing Simulation using Morse Simulator / Blender ")

<div class="center">

<div id="tab1">

|                         |                  |                     |
|:-----------------------:|:----------------:|:-------------------:|
|     **Comparison**      |   **Planners**   |                     |
|       **Metrics**       |    ***MPC***     |      ***TEB***      |
|        Lap time         |      39.81s      |       45.91s        |
| Maximum speed attained  |     1.3 m/s      |       1.3 m/s       |
| Maximum possible speed  |     2.0 m/s      |       1.3 m/s       |
| Cross Track Error (CTE) |       Low        |        High         |
|    Dynamic obstacles    | Unable to handle | Replans accordingly |
|     Reverse Driving     |      Unable      |        Able         |

Short Comparison Overview for Planners

</div>

<span id="tab1" label=" ">\[tab1\]</span>

</div>

## Structure

* software_integration

    * buildLibrealsense2TX(submodule) : Custom package to install Librealsens to TX2

    * Firmwares: Firmwares for different environments

        * arduino : Arduino scripts for interfacing the remote receiver

        * FourWheelCarConfigurations(VESC): VESC Configurations

        * SixWheelCar_Firmware: MCU Codes and compiled HEX files

    * JetsonPower: Module and user-space application for integrating LiPo battery readings into Linux

* morse: **[Morse](morse/Readme.md)** simulation environments
    * fourwd: Environment for the 4WD car
    * sixwd: Environment for the 6WD car

* Six_Wheeled_Motor_Controller: The files in the CD sent with Motor Controller

* src : All catkin based packages

    * ackermann_rc: Interface between an RC receiver connected to arduino and ROS

    * bear_car_launch: Collection of launch files that integrate most of the other packages here.

    * gscam(3rd party submodule):  Broadcasting any GStreamer camera source as ROS image. It is used to interface the onboad camera of the Jetson TX2 
                                   dev-board (rear camera)

    * hector_slam(3rd party submodule): 2D Lidar SLAM package that we use

    * imu_tools(custom submodule): Contains IMU filters 				

    * low_level_speed_controller: Speed Controllers

    * low_level_steering_controller : Steering Controller

    * odometry_agent : Configurations of sensor integrations

    * pid(3rd party submodule): General use PID package

    * pose_follower: Adjusted local planner implementation also containing many relevant launch files for the initialization of the entire move_base based 
                     navigation stack.

    * racecar(3rd party submodule): We are mostly reusing the contained achermann_cmd_mux package for interfacing VESC on the 4WD.

    * rc_msgs: Remote control ROS message definitions
    
    * realsense-ros: ROS interface to the Intel RealSense camera

    * rf2o_laser_odometry(3rd party submodule): The package that publishes odometry messages from Hector Slam

    * robot_localization(3rd party submodule): The package that contains EKF integrations that we use

    * rplidar_ros(3rd party submodule): ROS package of Laser Scanner

    * serial_6w: Serial Communication node to communicate with motor controller

    * sixwd_msgs : Custom message to interface six wheeled car's Controller

    * six_wheel_odom: The package that publishes odometry by six wheeled car's motor controllers

    * vesc(custom submodule): VESC interface for ROS and odometry
    
    ** New additional Packages

        * cartographer - SLAM package from Google

        * cartographer_ros - SLAM package ros interface

        * ceres-solver - Solver used by cartographer for optimisation

        * image_pipeline - various packages available to process the image inputs into required form.(we use this to convert depth input to PointCloud2)

        * low_level_controllers - used to interface various ros_controllers plugins by writing a robot hardware interface    

        * multi_goal - way point generator for race_track

        * navigation - classic navigation stack with move_base

        * navigation_experimental - Contains SBPL local planner as plugin for move_base

        * ros_controllers - Diffrent controller plugins to select from depending upon the type of joint or type of control desired. A hardware interface 
                            should be written to interface these controllers

        * robot_navigation - Navigation stack with Locomotor

        * steerforce_conv - custom package for conversion of linear and angular velocity commands to steer_force and steer_angle (only for simulation)

        * teb_local_planner - Local planner plugin for move_base
        
        * mpc_local_planner_mb - MPC Local planner custom plugin for move_base


## Clone and Build

First, create your self a SSH key to access the repository. To do so click to your avatar at the top right then click Settings after that choose SSH keys section at the right of the screen. There you will see a tutorial to generate a SSH key provided by Git-Lab.

After that you will be able to clone the repository

`git clone git@gitlab-edu.aot.tu-berlin.de:aaip_ws19_20/group4/software_integration.git`

Then also get submodules

`git submodule update --init --recursive`

Installing ROS & Catkin

ROS/catkin: Under 'src' folder we have many integrations for ROS. To properly install ROS please check https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment link. Please choose your version properly Jetson uses Ubuntu 18.04 and therefore the Melodic version.


### Installation

#### Dependencies

Automatic resolving of ROS dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Here are some package dependencies (they may not come with full desktop installation)
but you can first try without, but if you face any error please manually install and resolve dependencies:

- Ackermann (`sudo apt install ros-<version>-ackermann*`)
- gstreamer-1.0
- teleop_twist_joy (`sudo apt install ros-<version>-teleop-twist-joy`)
- teleop_twist_keyboard (`sudo apt install ros-<version>-teleop-twist-keyboard`)
- geographic-msgs (`sudo apt install ros-<version>-geographic-msgs`)
- serialConfig (`sudo apt install ros-<version>-serial*`) Only needed on the robot.
- realsense2-camera (`sudo apt install ros-<version>-realsense2-camera`) Only needed on the robot.

#### Build
NOTE: catkin_make does not support cartographer. Please make sure to install catkin tools and remove the devel and build folder before compiling.
NOTE: steer_force converter uses a simple pid library in python. Make sure you install the library suited for python 2.7.

To install Simple_pid:
```bash
pip install simple-pid
```

To install Ipopt solver for MPC:
* Detailed instructions in the `document` folder in the package
* To be followed as per the system architecture 

To install catkin_tools:
```bash

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

sudo apt-get update
sudo apt-get install python-catkin-tools
```

```bash
catkin build
```


## Run

For executing different software setups please refer to the linked platform specific Readmes.

1. **[4WD platform](RUNNING_FourWd.md)**  

### Architecture

Once the project is running, a way to better visualize the connection of the topics, nodes, transform frames is through using rqt package. Run `rqt` on the terminal. It provides many options to visualize under *Plugins* tab. Click on *Introspection-->Node Graph* to see how the ros nodes are connected, click on *Visualization-->TF Tree* to see transform frames. Please examine the other plugins to get more insight into the project.

## Sensors Setup

In our platforms we are using following sensors

- Wheel Encoder:

  Six Wheeled platform has its own wheel encoders for each wheels but they are only one phase encoders. Therefore they can't sense rotation direction, we solved this issue at odometry node assuming wheels never slides and rotates in commanded direction. Four wheeled system uses ERPM readings from BLDC motor. We also integrated an additional wheel encoder on it but not integrated it to ROS environment.

- Laser Scanner:

  Both cars have a RPLidar A3 laser scanner.

- IMU (Integrated on Camera):

  Intel D435i cameras have integrated IMU sensors on them. However, this IMU features only accelerometer and gyroscope and is missing a magnetometer.

- Camera: Front camera is the Intel D435i RGBD camera. Backcamera is the onboard MP CSI camera module on the Jetson TX2 dev-board (Omnivision OV5693)


