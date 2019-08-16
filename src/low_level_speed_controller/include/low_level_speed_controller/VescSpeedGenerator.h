#pragma once 


#include "low_level_speed_controller/SpeedCommandGeneratorBase.h"



class VescSpeedGenerator : public SpeedCommandGeneratorBase{
    public:
        CommandRequest createSpeedCommand(double speed_setpoint);
        ~VescSpeedGenerator(){}

    private:
};