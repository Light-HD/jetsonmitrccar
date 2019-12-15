import logging; logger = logging.getLogger("morse." + __name__)

from geometry_msgs.msg import Twist
from morse.middleware.ros import ROSSubscriber


class AckermannROS(ROSSubscriber):

    ros_class = Twist

    def update(self, message):
        # logger.info("Message Received: %s" % message.linear.x)
        self.data["force"] = -4*message.linear.x
        self.data["steer"] = 4*message.angular.z
