#include "low_level_speed_controller/SpeedCommandInterfaceBase.h"
#include "ros/ros.h"

SpeedCommandInterfaceBase::SpeedCommandInterfaceBase() : auto_execute_rate(1.0 / 10.0), node_handle("~"), command_timeout(1.0){
    ROS_INFO("Speed Interface Initialized");
    this->op_type = OperationType::MANUAL;
    this->max_acceleration = 0;
    this->max_speed = 0;
    this->min_speed = 0;
}

SpeedCommandInterfaceBase::SpeedCommandInterfaceBase(ros::NodeHandle &n) : SpeedCommandInterfaceBase(){
    node_handle = n;
    //callback_timer = node_handle.createTimer(ros::Rate(10.0), &SpeedCommandInterfaceBase::automatic_callback,this);
}


SpeedCommandInterfaceBase &SpeedCommandInterfaceBase::set_max_limits(double max_acc, double max_s, double min_s){
    this->max_acceleration = max_acc;
    this->max_speed = max_s;
    this->min_speed = min_s;

    return *this;
}

SpeedCommandInterfaceBase &SpeedCommandInterfaceBase::set_operation_type(SpeedCommandInterfaceBase::OperationType t){
    this->op_type = t;

    if(t == SpeedCommandInterfaceBase::OperationType::AUTOMATIC){
        callback_timer = node_handle.createTimer(auto_execute_rate, &SpeedCommandInterfaceBase::automatic_callback,this);
    }else{
        callback_timer.stop();
    }

    return *this;
}

SpeedCommandInterfaceBase& SpeedCommandInterfaceBase::set_command_timeout(ros::Duration new_dur){
    this->command_timeout = new_dur;

    return *this;
}

SpeedCommandInterfaceBase& SpeedCommandInterfaceBase::set_current_speed(double speed){
    this->current_speed = speed;

    return *this;
}

SpeedCommandInterfaceBase& SpeedCommandInterfaceBase::set_execution_rate(ros::Duration new_rate){
    this->auto_execute_rate = new_rate;

    callback_timer.setPeriod(this->auto_execute_rate);
    
    return *this;
}