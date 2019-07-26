#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <sixwd> environment

Feel free to edit this template as you like!
"""

from morse.builder import *
from sixwd.builder.robots import Sixwd
from sixwd.builder.actuators import Basicspeed

# Add the MORSE mascott, MORSY.
# Out-the-box available robots are listed here:
# http://www.openrobots.org/morse/doc/stable/components_library.html
#
# 'morse add robot <name> sixwd' can help you to build custom robots.
robot = Sixwd()

# The list of the main methods to manipulate your components
# is here: http://www.openrobots.org/morse/doc/stable/user/builder_overview.html
robot.translate(1.0, 0.0, 0.5)
robot.rotate(0.0, 0.0, 3.5)
robot.scale = [2.0,2.0,2.0]
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
# basic_actuate.properties(frequency = 10.0)

basic_actuate.add_stream('ros','sixwd.middleware.ros.basicspeed_ros.BasicSpeedROS')



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
env = Environment('sandbox', fastmode = False)
env.set_camera_location([-18.0, -6.7, 10.8])
env.set_camera_rotation([1.09, 0, -1.14])
env.simulator_frequency(20,5,5)