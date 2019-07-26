import logging; logger = logging.getLogger("morse." + __name__)

from geometry_msgs.msg import Twist
from morse.middleware.ros import ROSSubscriber
from morse.core import blenderapi

class BasicSpeedROS(ROSSubscriber):

    ros_class = Twist

    def update(self, message):
        #logger.info("Message Received: %s" % message.linear.x)
        self.data["XDot"] = message.linear.x
        self.data["YDot"] = message.angular.z

        if self.data["LastData"] < -2: 
            logger.error("Frequency mismatch Expected data at %s hz" % self.data["frequency"])
        self.data["LastData"] += 1