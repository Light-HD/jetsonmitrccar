#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <test> environment

Feel free to edit this template as you like!
"""

from morse.builder import *
from morse.sensors import *
from fourwd.builder.robots import Hummerscaled
from fourwd.builder.sensors.CustomBattery import Custombattery
import math

#Set Sensor Frequency
sense_freq = 100

robot = Hummerscaled()
robot.add_default_interface('ros')
scale = 0.2
robot.properties(scale=scale)
robot.properties(GroundRobot=True)
robot.name = "FourWD"
robot.scale = [scale, scale, scale]

# This is the wheel odometry tf
wheel_odom = Velocity()
# wheel_odom.frequency(sense_freq)
wheel_odom.add_stream('ros', frame_id="odom", topic="wheel_odom") #child_frame_id='base_link')
wheel_odom.translate(0.0, 0.0, 0.0)
wheel_odom.rotate(0.0, 0.0, 0)
# odom = Odometry()
# # odom.frequency(sense_freq)
# odom.add_stream('ros', frame_id="odom", topic="odometry/filtered", child_frame_id='base_link') #child_frame_id='base_link')
# odom.translate(0.0, 0.0, 0.0)
# odom.rotate(0.0, 0.0, 0)
# odom.alter('Noise', pos_std = 0.1, rot_std = math.radians(5))

# IMU sensor located inside the camera
imu = IMU()
imu.name = "imu"
imu.add_stream('ros', frame_id='camera_imu_optical_frame', topic='imu/data')
# imu.alter('Noise', pos_std = 0.1, rot_std = math.radians(5))
imu.translate(0.6, 0.0, 1.2)
imu.rotate(0.0, 0.0, 0.0)

# Add a pose sensor that exports the current location and orientation. Note that this is only for testing purposes
pose = Pose()
pose.add_stream('ros', frame_id="map", topic='pose')

# Laser scanner for 360 degree
laser_scanner = Hokuyo()
# laser_scanner.frequency(sense_freq)
laser_scanner.name = "laser_scan"
laser_scanner.add_stream('ros', frame_id="laser", topic="scan")
laser_scanner.translate(0.0, 0.0, 1.6)
laser_scanner.properties(resolution=0.5) #0.5 before
laser_scanner.properties(laser_range=25.0)
laser_scanner.properties(scan_window=360)
laser_scanner.properties(Visible_arc=False)
laser_scanner.rotate(0.0, 0.0, 0.0)
laser_scanner.create_laser_arc()

# add battery
battery = Custombattery()
battery.frequency(1)
battery.add_overlay('ros', 'fourwd.overlays.battery_overlay.RandomInitBatteryOverlay')
# Properties below might be changed after the experiments or according to the scenarios (how fast the batteries are drained)
battery.properties(DischargingRate = 0.05, ChargingRate = 2.0, MotorDrainingRate = 0.15)
battery.add_stream('ros', 'morse.middleware.ros.battery.Float32Publisher')

# RGBD camera
kinect = Kinect()
kinect.depth_camera.add_stream('ros', frame_id="camera_depth_frame", topic='/camera/depth', topic_suffix='/image_raw')
kinect.video_camera.add_stream('ros', frame_id="camera_color_frame", topic='/camera/rgb', topic_suffix='/image_raw')
kinect.translate(0.6, 0, 1.2)
kinect.rotate(0.0, 0.0, 0)

# Rear camera
rgba_camera = VideoCamera() # Rear camera?
rgba_camera.add_stream('ros', frame_id="camera_rear", topic='/camera_rear/', topic_suffix='/image_raw') #TODO: the frame_id of the cameras need to be linked to /camera_link
rgba_camera.rotate(0, math.pi, math.pi)
rgba_camera.translate(-3.3, 0, 1)


# The list of the main methods to manipulate your components
# is here: http://www.openrobots.org/morse/doc/astable/user/builder_overview.html
robot.translate(-5.8, 0.0, 0.2)
robot.rotate(0.0, 0.0, 0.0)
robot.set_mass(1.5)

# Add a motion controller
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#actuators
#
# 'morse add actuator <name> test' can help you with the creation of a custom
# actuator.
steerforce = SteerForce()
steerforce.add_stream('ros', 'fourwd.middleware.ros.ackermann_ros.AckermannROS', topic='morse_steerforce_cmd')
steerforce.translate(0, 0, 0)
steerforce.rotate(0, 0, 0)
# motion = MotionXYW()
# motion.add_interface('ros', topic='cmd_vel')
# place your component at the correct location


robot.append(imu)
robot.append(laser_scanner)
# robot.append(motion)
robot.append(steerforce)
robot.append(wheel_odom)
# robot.append(odom)
robot.append(rgba_camera)
robot.append(kinect)
robot.append(pose)
robot.append(battery)

# a basic keyboard controller for testing purposes
keyboard = Keyboard()
robot.append(keyboard)

# Adding a charging zone
charging_station = PassiveObject('fourwd/environments/charging_station.blend', 'ChargingStation')
angle = math.pi
tray_x = 1.6
tray_y = -2.5
charging_station.translate(tray_x, tray_y, 0.0)
charging_station.rotate(0.0, 0.0, angle)
# define charging zone
charging_zone = Zone(type = 'Charging')
charging_zone.size = [0.5, 0.5, 1.0]
# compute offset from tray center also taking rotation into account
xoff = 0.75 * math.cos(angle) - 0.0 * math.sin(angle)
yoff = 0.0 * math.cos(angle) +  0.75 * math.sin(angle)

charging_zone.translate(tray_x + xoff, tray_y + yoff, 0.0)

# set 'fastmode' to True to switch to wireframe mode
env = Environment('fourwd/environments/CleanEnv.blend',fastmode = False)
env.set_camera_location([-18.0, -6.7, 10.8])
env.set_camera_rotation([1.09, 0, -1.14])
env.properties(latitude=1.53, longitude=45.1, altitude=0.0)
env.set_viewport(viewport_shade='TEXTURED', clip_end=1000)
env.use_internal_syncer()
env.show_framerate(True)
env.add_stream('ros')
