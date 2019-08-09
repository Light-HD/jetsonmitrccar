#pragma once

#include <type_traits>
#include "low_level_speed_controller/SpeedCommandInterface.h"
#include <memory>

class SpeedCommandInterface;
class SpeedCommandGenerator;

class SpeedTester : public SpeedCommandInterface{

};

template<class TCommandGen, class TSpeedInterface>
class LowLevelSpeedController{
    public:
        static_assert(std::is_base_of<SpeedCommandInterface, TSpeedInterface>::value,"Class should inherit SpeedCommandInterface");
        static_assert(std::is_base_of<SpeedCommandGenerator, TCommandGen>::value,"Class Should inherit SpeedCommandInterface");

    protected:
        std::unique_ptr<TCommandGen> commandGenerator;
        std::unique_ptr<TSpeedInterface> speedInterface;
};
