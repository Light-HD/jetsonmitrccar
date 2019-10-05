# software_integration

Integration repository for all relevant software pieces/submodules.
This project normally runs on Nvidia jetsonTX2 PCs on the cars (already installed).
If you would like to run some of the packages on your PC, please refer to the Build section.


##Questions/TODOs in/about submodules/directories

1. buildRealsenseTX2 did we change anything here? Why not a submodule if not?(Added Ass Submodule -Goksu)
2. FourWheelCarConfigurations(VESC) needs README(Briefly Explained How to Use Them -Göksu)
3. SixwheelFırmware needs README(Added - Göksu)
4. arduino has TODO in Readme(Ege knows it better)
5. jetsonTX2Power TODOS in Readme 
6. Where is the 4WD simulation environment?
7. Morse basicspeed.py is missing code docu, currently mostly boilerplate docu.
8. TODO add the end in all of your code either document why codes are commented or remove commented code.
9. TODO all of your packages require a README that exlains what it is
10. low_level_speed_controller fails during compilation.


## Structure

**TODO Describe repo structure, all included packages have to be mentioned and for what are they responsible, how should/can they be used briefly, more detailed information should be referenced in a subreadme within the corresponding package/module. Here, you should also make clear what is just a git submodule from someone else and what you/we have been developed** (Added the ones that I know but the design is really bad if you have any recomendations for mark down format ı wold be happy - Göksu)

-software_integration

----buildLibrealsense2TX : The Package to install Librealsens to TX2

----Firmwares: Firmwares for different environments

------------------arduino : Arduino Scrips

------------------FourWheelCarConfigurations(VESC): VESC Configurations

------------------SixWheelCar_Firmware: MCU Codes and compiled HEX files

----JetsonPower:

----morse

----Six_Wheeled_Motor_Controller: The files in the CD sent with Motor Controller

----src : All catkin based packages

------------------ackermann_rc

------------------conde

------------------gscam

------------------hector_slam: Slam package that we use

------------------imu_tools: Contains IMU filters 				

------------------low_level_speed_controller: Speed Controllers 

------------------low_level_starter: The launch file to easily start six wheeled platform

------------------low_level_steering_controller : Steering Controller 

------------------morse_teleop : 

------------------odometry_agent : Configurations of sensor integrations

------------------pid : General use PID package

------------------pose_follower 

------------------racecar

------------------racecar_gazebo

------------------racecar_simulator

------------------rc_msgs

------------------rf2o_laser_odometry : The package that publishes odometry messages from Hector Slam

------------------robot_localization: The package that contains EKF integrations that we use

------------------rplidar_ros: ROS package of Laser Scanner

------------------rs_cam_and_imu_filter_launch: Launch file to start imu filter and sensor

------------------serial_6w: Serial Communication node to communicate with motor controller

------------------sixwd_msgs : Custom message to interface six wheeled car's Controller

------------------six_wheel_odom: The package that publishes odometry by six wheeled car's motor controllers

------------------vesc: VESC interface for ROS and odometry 


## Build

**TODO Describe cloning (recursive) + dependency resolving/installs + build** (I added the way that I use - Göksu )

First, create your self a SSH key to access the repository. To do so click to your avatar at the top right then click Settings after that choose SSH keys section at the right of the screen. There you will see a tutorial to generate  a SSH key provided by Git-Lab.

After That you will be able to clone the repository 

`git clone git@gitlab-edu.aot.tu-berlin.de:small_autonomous_driving/software_integration.git `

Change to our development branch

`git checkout patch-1`

Then also get submodules

`git submodule init`

`git submodule update`

**TODO This should also reflect the different environments such as Arduino, ROS, Linux Kernel modules, AVR for 6WD motorcontoller ...**(I briefly explained the environments that I used - Göksu)

There are several environments the project consists of.

1. ROS: Under 'src' folder we have many integrations for ROS. To properly install ROS please check https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment link. Please choose your version properly Jetson uses 18.04 ubuntu and therefore they uses Melodic version.
2. Arduino
3. Linux Kernel
4. AVR: Note that to code the MCU's on Motor Controller of six wheeled platform we needed to change the codes on the motor controller. To dose so I strongly recommend Atmel Studio. There you can write your code and at the build menu you can compile them to .hex format. You can also upload these codes to the MCU's by using Device Programming Menu. Check Firmwares/SixWheelCar_Firmware folder.
5. VESC Tool: Vesc tool is the GUI of the VESC. There you can change motor controlling parameters easily. Check Firmwares/FourWheeledCarConfigurations(Vesc) Folder.

**TODO here we could also differentiate between what has to be build on the car and what on the developer machine? SHOULD we create two integration repositories for this?** << This would also save a lot of space on the jetson because there we don't need simulation etc.

**TODO is this enough for resolving dependencies?**

```bash
rosdep install --from-paths src --ignore-src -r -y
```
### Installation

To install the ros project, some packages that might be missing (may not come with full desktop installation):
- Ackermann (sudo apt install ros-<version>-ackermann*)
- gstreamer-1.0
- teleop_twist_joy (sudo apt install ros-<version>-teleop-twist-joy)
- teleop_twist_keyboard (sudo apt install ros-<version>-teleop-twist-keyboard)
- geographic-msgs (sudo apt install ros-<version>-geographic-msgs)
- serialConfig (sudo apt install ros-<version>-serial*)
- opencv

## Run

### Running Four Wheel
** TODO Describe how to run the code, potentially explaining different launch files for different purposes**

** TODO Different setups are, for instance remote controlled driving, move_base-based driving, different simulation engines.

### Running Six Wheel
blabla


## Other Documentations

** TODO add the links of the other README files for each package
**TODO if appropriate link READMEs of subdirectories/modules below**


## Sensors Setup

In our platforms we are using following sensors

- Wheel Encoder:

  Six Wheeled platform has its own wheel encoders for each wheels but they are only one phase encoders. Therefore they can't sense rotation direction, we solved this issue at odometry node assuming wheels never slides and rotates in commanded direction. Four wheeled system uses ERPM readings from BLDC motor. We also integrated an additional wheel encoder on it but not integrated it to ROS environment.

- Laser Scanner:

  Both cars have their RPLidar A3 laser scanner.

- IMU (Integrated on Camera):

  Intel D435i cameras have integrated IMU sensors on them.

- Camera:

  For  many further applications we also have a Depth Camera on  platforms.


## Open Issues
 1. For VESC We applied Encoder that we can read it from VESC Firmware. To more accurate speed readings, ıt should also be implemented for ROS. Note that Vesc only reads
 encoder as rotor position. Therefore, two there exists two steps add position reading to VESC_Driver node Convert it into speed.
  2. Motor controller for six wheeled platform' motor controller node (serial_6w) needs a launch file to easily change parameters especially USB device name.
  3. 6 Wheeled platform's encoder readings is not that reliable. For now we are using rf2o node for odometry. Wheel odometry data my be improved 
  4. 6 Wheeled platform's one motor has damaged. It should be changed. Please check the issue .
 5. This project normally runs on Nvidia jetsonTX2 PCs on the cars (already installed).
     If you would like to run some of the packages on your PC, please refer to the Build section.
 6. PID performances can be improved to see the interface please look interface section.
  7. 6 Wheeled platform's one motor has damaged. I should be changed. Please check the related issue .
  8. Maybe first developement srep can be adding visual odometry to the cars.

#FAQ

**TODO if you have collect solutions for common pitfalls (maybe things you experienced yourself)**
