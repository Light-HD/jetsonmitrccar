import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.actuator
from morse.builder import Clock

from morse.core import blenderapi

from morse.core.services import service, async_service, interruptible
from morse.core import status
from morse.helpers.components import add_data, add_property

class Basicspeed(morse.core.actuator.Actuator):
    """Write here the general documentation of your actuator.
    It will appear in the generated online documentation.
    """
    _name = "Basicspeed"
    _short_desc = "Basic Speed Controller Incorporating Dynamics"

    # define here the data fields required by your actuator
    # format is: field name, initial value, type, description
    add_property("Velocity", 0, "Velocity","double", "Velocity of the car")
    add_property("WheelBase",0,"WheelBase","double","WheelBase of the car")
    add_property("Mass",0,"Mass","double","Mass of the car")
    add_property("MaxSpeed",0,"MaxSpeed","double","Max Speed car can achieve")
    add_property("MaxAngularSpeed",0,"MaxAngularSpeed","double","Max Angular Speed car can achieve")
    add_property("Acceleration",0,"Acceleration","double","Acceleration When in Acceleration Mode")
    add_property("AngularAcc",0,"AngularAcc","double","Angular Acceleration When in Acceleration Mode")
    add_property("Mode","Velocity","Mode","str","Control Mode:Velocity or Acceleration")
    
    #add_data("X",0,"double","X position of the car")
    #add_data("Y",0,"double","Y position of the car")
    #add_data("Theta",0,"double","Yaw angle of the car")
    add_data("XDot",0,"double","Velocity in X direction")
    add_data("YDot",0,"double","Velocity in Y direction")
    add_data("LastData",0,"double","Last Time a Data is received from ROS")
    add_data("frequency",0,"double","Actuator Frequency")
    #add_data("ThetaDot",0,"double","Angular Velocity")

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)
        self.decay = 0.9
        self.local_data["frequency"] = self._frequency
        # Do here actuator specific initializations
        logger.info('Component initialized at %d' % self._frequency)

    @service
    def get_position(self):
        """ This is a sample service.

        Simply returns the value of the internal counter.

        You can access it as a RPC service from clients.
        """

        return self.local_data['X']

    @interruptible
    @async_service
    def async_test(self, value):
        """ This is a sample asynchronous service.

        Returns when the internal counter reaches ``value``.

        You can access it as a RPC service from clients.
        """
        self._target_count = value

    def default_action(self):
        """ Main loop of the actuator.
        
        Implements the component behaviour
        """

        keyboardController = blenderapi.keyboard()
        active_state = blenderapi.input_active()

        linear_change = 0.0
        angular_change = 0.0

        local_x = self.local_data['XDot']
        local_y = self.local_data['YDot']

        # self.local_data['XDot'] = self.local_data['YDot'] = 0

        
        #if self.Mode == "Acceleration":
            # logger.info("Acc Mode")
        if keyboardController.events[blenderapi.UPARROWKEY] == active_state:
            linear_change += self.Acceleration

        if keyboardController.events[blenderapi.DOWNARROWKEY] == active_state:
            linear_change -= self.Acceleration

        if keyboardController.events[blenderapi.LEFTARROWKEY] == active_state:
            angular_change += self.AngularAcc

        if keyboardController.events[blenderapi.RIGHTARROWKEY] == active_state:
            angular_change -= self.AngularAcc
        

        local_x += linear_change
        local_y += angular_change

        if local_x < 0:
            local_x = max(local_x,-self.MaxSpeed)
        else:
            local_x = min(local_x,self.MaxSpeed)

        if local_y < 0:
            local_y = max(local_y,-self.MaxAngularSpeed)
        else:
            local_y = min(local_y,self.MaxAngularSpeed)
            
            

        if linear_change == 0:
            local_x *= self.decay
                #if self.local_data['XDot'] < 0:
                #    self.local_data['XDot'] += self.Acceleration
                #if self.local_data['XDot'] > 0:
                #    self.local_data['XDot'] -= self.Acceleration

        if abs(local_x) < 0.005:
            local_x = 0.0

        if angular_change == 0:
            local_y *= self.decay
                #if self.local_data['YDot'] < 0:
                #    self.local_data['YDot'] += self.AngularAcc
                #if self.local_data['YDot'] > 0:
                #    self.local_data['YDot'] -= self.AngularAcc

        if abs(local_y) < 0.005:
            local_y = 0.0

            # logger.info("YDot: %s" % self.local_data['YDot'])
            # check if we have an on-going asynchronous tasks...
        self.local_data['XDot'] = local_x
        self.local_data['YDot'] = local_y
        self.local_data["LastData"] -= 1
        self.robot_parent.apply_speed('Position',[local_x ,0 ,0],[0,0,local_y / 2.0])
