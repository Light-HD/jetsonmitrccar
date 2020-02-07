source devel/setup.bash
bash parallel "roslaunch bear_car_launch autonomous_navigation_fourwd.launch is_simulation:=false use_ros_control:=true" 
