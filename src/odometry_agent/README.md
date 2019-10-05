# odometry_agent

In this package you can find many configurations about our odometry sources. 

At odometry_agent.launch you can choose which sensors do you want to use or fuse and which care are you using.

At  four/six_wheel_transforms.launch you can find transforms for sensors. Note that thay are not placed precisely. 

hector_configuration.launch and slam_components.launch sets parametetrs about hector_slam and starts slam algorithms. real_world_tf.launch aslo contains transforms related to hector_slam.

Note that the most critical part of the package is EKF which publishes odom to base_link transform. 

The launch file starts it is ekf_template.launch and the related parameters and really detailed exlanations about them inside the pram/ekf_template.yaml file.

Note that in six wheeled car wheel odometry is not that reliable. so we need to decrease its weigh. Also Four wheeled car's wheel odometry is better than its laser scan odometry. To change their weigh we have two formula fist of all if yo wish you can close some of them by just deleting their name from pram/ekf_template.yaml file. The secon way is more convenient but harder to implement you can just basicly change covarience values at packages. Note that smaler covariance means more precission. And on Rviz the position of your robot should always be in the covariance elipse. The covariences of the sensors are inside the files ;

vesc/vesc_ackermann/src/vesc_to_odom.cpp

six_wheel_odom/src/six_wheel_odom.cpp

rf2o_laser_odometry/src/CLaserOdometry2DNode.cpp