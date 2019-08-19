# software_integration

Integration repository for all relevant software pieces/submodules

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
