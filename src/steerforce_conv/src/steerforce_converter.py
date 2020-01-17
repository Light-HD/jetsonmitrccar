#! /usr/bin/env python

import rospy
from geometry_msgs.msg import *
from simple_pid import PID
import math
    
force_pub = rospy.Publisher("morse_steerforce_cmd", Twist, queue_size=1)
DEBUG = False

#Fetch cmd_vel msgs and set it as target velocities
def cmd_vel_callback(msg):
    global ref
    if DEBUG:
        rospy.loginfo("Received a /cmd_vel message! V(x),V(y): [%f, %f] Theta(z):[%f]"%(msg.linear.x, msg.linear.y,msg.angular.z))
    ref = msg
    pid.setpoint = ref.linear.x

#Fetch wheel_odom msgs and set it as feedback for PID    
def wheel_odom_callback(msg):
    global fbk
    if DEBUG:
        rospy.loginfo("Received a /odom message! V(x),V(y): [%f, %f] Theta(z):[%f]"%(msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z))
    fbk = msg
    
#Main PID loop to calculate equivalent force and heading from cmd_vel msgs
def pidloop(ref, fbk):
    
    #Calculate Steering angle 
    wheel_base = rospy.get_param("/steerforce_converter/wheel_base") 
    if(ref.angular.z == 0 or ref.linear.x == 0):
        heading = 0
    else: 
        heading = math.atan2(wheel_base * ref.angular.z, ref.linear.x)
    heading = max(min(heading, rospy.get_param("/steerforce_converter/max_steer_ang")), -rospy.get_param("/steerforce_converter/max_steer_ang"))
    if DEBUG:
        rospy.loginfo("Ref: Theta [%f] Calculated Heading:[%f]"%(ref.angular.z,heading))
    
    #Calculate Linear Force
    lin_force = pid(fbk.twist.linear.x)
    if DEBUG:
        rospy.loginfo("Feedback: Vel [%f] Control Force:[%f]"%(fbk.twist.linear.x,lin_force))

    #Publish Steerforce Command    
    final_control = Twist()
    final_control.linear.x = lin_force
    final_control.angular.z = heading
    force_pub.publish(final_control)
    
def listener():
    rospy.init_node('steerforce_converter')
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    rospy.Subscriber("/wheel_odom", TwistStamped, wheel_odom_callback)  
    rate = rospy.Rate(2000)
    while not rospy.is_shutdown():
       # rospy.loginfo("PID_Called")
        if DEBUG:
            rospy.loginfo("Received a ref message! V(x),V(y): [%f, %f] Theta(z):[%f]"%(ref.linear.x, ref.linear.y, ref.angular.z))
        pidloop(ref,fbk)
        rate.sleep()

if __name__ == '__main__':
    global ref, fbk
    pid_gains = rospy.get_param("/steerforce_converter/pid_gains")
    ref = Twist()
    fbk = TwistStamped()
    pid = PID(pid_gains['kp'], pid_gains['ki'], pid_gains['kd'], setpoint = ref.linear.x)
    pid.sample_time = 0.001
    pid.output_limits = (-1, 1)
    listener()


   
