# bear_car_launch

This package collects various system relevant launch files for both platform 6WD and 4WD.

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


