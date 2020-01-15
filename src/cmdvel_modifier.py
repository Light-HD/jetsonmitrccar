#! /usr/bin/env python
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from simple_pid import PID

# force_pub = rospy.Publisher("control", Twist, queue_size=50)
def callback0(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

def callback1(msg):
    rospy.loginfo((msg.twist.linear.x))



def listener():
    rospy.init_node('cmd_vel_listener')
    a = rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.init_node('oodometry', anonymous=True)
    b = rospy.Subscriber("/odometry_filtered", Odometry, pidloop )
    return a,b
    rospy.spin()


if __name__ == '__main__':
    a,b = listener()