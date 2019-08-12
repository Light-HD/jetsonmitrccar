#include "low_level_speed_controller/VescSpeedGenerator.h"



CommandRequest VescSpeedGenerator::createSpeedCommand(double input_setpoint){

    CommandRequest command;
    switch(control_type){
        case SpeedCommandGeneratorBase::ControlType::Acceleration:
        {
            input_setpoint = (input_setpoint > 0.0) ? 
                (clamp_value(input_setpoint, 0.0, max_acc_limit)) : 
                    (-clamp_value(-input_setpoint, 0.0, max_acc_limit));

            double setpoint = input_setpoint + current_speed;
            setpoint = (setpoint > 0.0) ? 
                (clamp_value(setpoint,max_linear_speed,min_linear_speed)) : 
                    (-clamp_value(-setpoint, max_linear_speed, min_linear_speed));

            command.value = setpoint;
            command.req_type = CommandRequest::RequestType::SPEED;
            break;

        }
            
        case SpeedCommandGeneratorBase::ControlType::Speed:
        {
            input_setpoint = (input_setpoint > 0.0) ? 
                (clamp_value(input_setpoint,max_linear_speed,min_linear_speed)) : 
                    (-clamp_value(-input_setpoint, max_linear_speed, min_linear_speed));
   
            command.value = input_setpoint;
            command.req_type = CommandRequest::RequestType::SPEED;
            break;
        }
            
    }
    
    return command;
}