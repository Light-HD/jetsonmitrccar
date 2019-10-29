#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <test> environment

Feel free to edit this template as you like!
"""

from morse.builder import *
from morse.sensors import *
from fourwd.builder.robots import Hummerscaled

robot = Hummerscaled()
robot.add_default_interface('ros')
robot.properties(scale = 0.2)
robot.properties(GroundRobot = True)
robot.name = "Hummer"
robot.scale = [0.2, 0.2, 0.2]

odom = Odometry()
odom.add_stream('ros',child_frame_id="base_link",topic="wheel_odom")
odom.translate(0.0, 0.0, 0.0)
odom.rotate(-1.57, -1.57, 1.57)
# odom.level("differential")

imu = IMU()
imu.name = "imu"
imu.add_stream('ros', topic='imu/data')
imu.translate(0.0, 0.0, 0.0)
imu.rotate(0.0, -1.57, 0.0)

# Add a pose sensor that exports the current location and orientation
pose = Pose()
pose.add_stream('ros', topic='pose')

# place your component at the correct location

laser_scanner = Hokuyo()
laser_scanner.name = "laser_scan"
laser_scanner.add_stream('ros',child_frame_id="/base_laser_link",topic="scan")
laser_scanner.translate(0, 0.2, 1.6)
laser_scanner.properties(resolution = 1.0) #0.5 before
laser_scanner.properties(laser_range = 30.0) #5.0 before
laser_scanner.properties(scan_window = 360)
laser_scanner.properties(Visible_arc = False)
laser_scanner.rotate(0.0, 0.0, 0.0)
laser_scanner.create_laser_arc()

kinect = Kinect()
kinect.depth_camera.add_interface('ros', topic='/camera/depth', topic_suffix='/image_raw')
kinect.video_camera.add_interface('ros', topic='/camera/rgb', topic_suffix='/image_raw')
kinect.translate(0, 0.6, 1.2)
kinect.rotate(0.0, 0.0, 1.57)

rgba_camera = VideoCamera() # Rear camera?
rgba_camera.add_stream('ros')
rgba_camera.translate(0, -3.3, 1)
rgba_camera.rotate(1.57, 3.14, 3.14)

# The list of the main methods to manipulate your components
# is here: http://www.openrobots.org/morse/doc/astable/user/builder_overview.html
robot.translate(1.0, 0.0, 0.5)
robot.rotate(0.0, 0.0, 0.0)
robot.set_mass(0.1)

# Add a motion controller
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#actuators
#
# 'morse add actuator <name> test' can help you with the creation of a custom
# actuator.
steerforce = SteerForce()
steerforce.add_stream('ros','fourwd.middleware.ros.ackermann_ros.AckermannROS')
# place your component at the correct location
steerforce.translate(0, 0, 0)
steerforce.rotate(0, 0, 0)

robot.append(imu)
robot.append(laser_scanner)
robot.append(steerforce)
robot.append(odom)
robot.append(rgba_camera)
robot.append(kinect)
robot.append(pose)


# To ease development and debugging, we add a socket interface to our robot.
#
# Check here: http://www.openrobots.org/morse/doc/stable/user/integration.html
# the other available interfaces (like ROS, YARP...)



# set 'fastmode' to True to switch to wireframe mode
# env = Environment('environments/indoor.blend', fastmode = False)
env = Environment('fourwd/environments/test_last.blend',fastmode = False)
env.set_camera_location([-18.0, -6.7, 10.8])
env.set_camera_rotation([1.09, 0, -1.14])
env.properties(latitude=1.53, longitude=45.1, altitude=0.0)
env.set_viewport(viewport_shade='TEXTURED', clip_end=1000)
env.show_framerate(True)
env.add_stream('ros')
