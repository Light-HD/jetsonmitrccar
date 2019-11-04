#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <sixwd> environment

Feel free to edit this template as you like!
"""

from morse.builder import *
from morse.sensors import *
from sixwd.builder.robots import Sixwd
from sixwd.builder.actuators import Basicspeed

# Add the MORSE mascott, MORSY.
# Out-the-box available robots are listed here:
# http://www.openrobots.org/morse/doc/stable/components_library.html
#
# 'morse add robot <name> sixwd' can help you to build custom robots.
robot = Sixwd()
robot.add_default_interface('ros')
robot.properties(scale = 1.5)
robot.scale = [1.5, 1.5, 1.5]

odom = Odometry()
odom.translate(0.0, 0.0, 0.0)
odom.rotate(0.0, 0.0, 0.0)
# odom.level("differential")

imu = IMU()
imu.name = "imu"
imu.translate(0.0, 0.0, 0.0)
imu.rotate(0.0, -1.57, 0.0)


# place your component at the correct location


laser_scanner = Hokuyo()
laser_scanner.name = "laser_scan"
laser_scanner.properties(resolution = 0.5)
laser_scanner.translate(0.0, 0.0, 0.135)
laser_scanner.properties(laser_range = 5.0)
laser_scanner.properties(scan_window = 360)
laser_scanner.properties(Visible_arc = False)
laser_scanner.rotate(0.0, 0.0, 1.57)
laser_scanner.create_laser_arc()

kinect = Kinect()

kinect.translate(0.175,0.0,0.1)
kinect.rotate(0.0, 0.0, 0.0)

rgba_camera = VideoCamera()
rgba_camera.translate(-0.175,0.0,0.2)
rgba_camera.rotate(0.0, 3.14, 1.57)

# The list of the main methods to manipulate your components
# is here: http://www.openrobots.org/morse/doc/stable/user/builder_overview.html
robot.translate(1.0, 0.0, 0.5)
robot.rotate(0.0, 0.0, 1.57)
robot.set_mass(2.0)

# Add a motion controller
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#actuators
#
# 'morse add actuator <name> sixwd' can help you with the creation of a custom
# actuator.
basic_actuate = Basicspeed()

basic_actuate.properties(Acceleration = 0.007)
basic_actuate.properties(AngularAcc = 0.006)
basic_actuate.properties(MaxAngularSpeed = 0.1)
basic_actuate.properties(MaxSpeed = 0.2)
basic_actuate.properties(Mode = "Acceleration")
basic_actuate.properties(decay = 0.9)
# basic_actuate.properties(frequency = 10.0)

basic_actuate.add_stream('ros','sixwd.middleware.ros.basicspeed_ros.BasicSpeedROS')

robot.append(imu)
robot.append(laser_scanner)
robot.append(odom)
robot.append(rgba_camera)
robot.append(kinect)
robot.append(basic_actuate)

# Add a keyboard controller to move the robot with arrow keys.
# keyboard = Keyboard()
# robot.append(keyboard)
# keyboard.properties(ControlType = 'Position')

# Add a pose sensor that exports the current location and orientation
# of the robot in the world frame
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#sensors
#
# 'morse add sensor <name> sixwd' can help you with the creation of a custom
# sensor.
pose = Pose()
robot.append(pose)

# To ease development and debugging, we add a socket interface to our robot.
#
# Check here: http://www.openrobots.org/morse/doc/stable/user/integration.html
# the other available interfaces (like ROS, YARP...)
# robot.add_default_interface('socket')
# basic_actuate.add_interface('ros',topic = "/test")

robot.add_default_interface('ros')

# set 'fastmode' to True to switch to wireframe mode
env = Environment('sixwd/environments/last.blend', fastmode = False)
env.set_camera_location([-18.0, -6.7, 10.8])
env.set_camera_rotation([1.09, 0, -1.14])
env.properties(latitude=1.53, longitude=45.1, altitude=0)
env.set_viewport(viewport_shade='TEXTURED', clip_end=1000)
#env.simulator_frequency(20,5,5)

env.show_framerate(True)

robot.add_default_interface('ros')
pose.add_stream('ros')
imu.add_stream('ros')
laser_scanner.add_stream('ros',topic="/base_scan")
odom.add_stream('ros',child_frame_id="/base_link",topic="/odom")
rgba_camera.add_stream('ros')
kinect.add_stream('ros')
