# odometry_agent


This package is only meant to provide the necessary sensor data concerning odometery to the EKF package which will fuse these data and publish odom to base_link transform and the processed odometry data is published on the topic `\odometry\filtered`

- Necessary data sources can be chosen
- parameter configuaration `param/ekf_template.yaml`  file for the ekf_node should also be edited incase of any changes made to the `odometery_agent` launch file.
