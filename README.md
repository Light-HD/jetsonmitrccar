# software_integration

Integration repository for all relevant software pieces/submodules

##Questions/TODOs in/about submodules/directories

1. buildRealsenseTX2 did we change anything here? Why not a submodule if not?
2. FourWheelCarConfigurations(VESC) needs README
3. SixwheelFırmware needs README
4. arduino has TODO in Readme
5. jetsonTX2Power TODOS in Readme
6. Where is the 4WD simulation environment?
7. Morse basicspeed.py is missing code docu, currently mostly boilerplate docu.
8. TODO add the end in all of your code either document why codes are commented or remove commented code.
9. TODO all of your packages require a README that exlains what it is
10. low_level_speed_controller fails during compilation.

2. 

**TODO if appropriate link READMEs of subdirectories/modules below**



## Structure

**TODO Describe repo structure, all included packages have to be mentioned and for what are they responsible, how should/can they be used briefly, more detailed information should be referenced in a subreadme within the corresponding package/module. Here, you should also make clear what is just a git submodule from someone else and what you/we have been developed**


## Build

**TODO Describe cloning + dependency resolving/installs + build**

**TODO This should also reflect the different environments such as Arduino, ROS, Linux Kernel modules, AVR for 6WD motorcontoller ...**

**TODO here we could also differentiate between what has to be build on the car and what on the developer machine? SHOULD we create two integration repositories for this?** << This would also save a lot of space on the jetson because there we dont need simulation etc.

**TODO is this enough for resolving dependencies?**
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Run

** TODO Describe how to run the code, potentially explaining different launch files for different purposes**

** TODO Different setups are, for instance remote controlled driving, move_base-based driving, different simulation engines.

## Sensors Setup

** TODO potentially add information for other sensors***

### Real Sense

rs_cam_and_imu_filter_launch is the launch file for launching realsense camera and imu filter together. Output is in the topic /imu/data and it can be seen with rviz plugin
 that is in imu_tools packages(can be installed with $sudo apt-get install ros-melodic-imu-tools but I suggest to build it from source since we are going to change the source code).
Do not forget to install imu_tools. 

Below is the link for to know how to calibrate the realsense D435i's imu.(Imu is not calibrated in the production line) Calibration is done with the python script
that is already in realsense_ros package in folder rs-imu-calibration. 
[https://www.intel.com/content/www/us/en/support/articles/000032303/emerging-technologies/intel-realsense-technology.html](url)


For calibration you need pyrealsense2, and you should build it from source in the following link
[https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python#building-from-source](url)
since jetson has a arm core which doesnt support pip.

 quaternion_to_yaw is used to obtain yaw angle. Imu filter madgwick publishes data as quaternion but we need yaw angle to know the vehicles orientaion, so it simply subscribes
 to /imu/data topic from taken from filter and publishes the yaw angle as Float64 to /yaw topic.
 
 The yaw angle was drifting due to gyro drift and it is handled with increasing "zeta" in the filter which adds/substracts a value for each time instant from the reading which 
 makes the yaw angle almost constant for long time. However, the main code handles this issue with the help of magnetometer which we don't have. Therefore, we changed the code 
 a little bit to work fine without magnetometer. New imu filter madgwick folder is in repo and you need to build it from source to get drift compensation working without magnetometer.

#FAQ

**TODO if you have collect solutions for common pitfalls (maybe things you experienced yourself)**

