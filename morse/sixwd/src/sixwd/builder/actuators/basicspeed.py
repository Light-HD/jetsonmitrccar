from morse.builder.creator import ActuatorCreator

class Basicspeed(ActuatorCreator):
    _classpath = "sixwd.actuators.basicspeed.Basicspeed"
    _blendname = "basicspeed"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

