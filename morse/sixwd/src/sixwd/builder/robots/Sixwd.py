from morse.builder import *

class Sixwd(GroundRobot):
    """
    A template robot model for Sixwd, with a motion controller and a pose sensor.
    """
    def __init__(self, name = None, debug = True):

        # Sixwd.blend is located in the data/robots directory
        GroundRobot.__init__(self, 'sixwd/robots/Sixwd.blend', name)
        self.properties(classpath = "sixwd.robots.Sixwd.Sixwd")
