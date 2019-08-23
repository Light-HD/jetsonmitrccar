Arduino Code To Receive and publish steering and throttle commands from RC controller.
Please make sure to setup rosserial_arduino package correctly.

**TODO add link description on how to setup!**

Usage:
After rosserial is setup, run Arduino code then restart Arduino.
Run: 
    rosrun rosserial_python serial_node.py /dev/ttyUSB0
with correct port number. Code requires the output of RC controller to be on a interrupt capable pin. Output of the RC controller can be directly connected.


To add Custom Messages to Arduino,please refer to http://wiki.ros.org/rosserial_arduino/Tutorials/Adding%20Custom%20Messages

rc_msgs package is a ROS package. Please make sure to copy to catkin workspace and catkin_make.

**TODO script/commands for copying**