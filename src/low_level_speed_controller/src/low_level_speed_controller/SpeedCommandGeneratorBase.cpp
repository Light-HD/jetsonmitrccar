#include "low_level_speed_controller/SpeedCommandGeneratorBase.h"
#include "ros/ros.h"

SpeedCommandGeneratorBase::SpeedCommandGeneratorBase() : node_handle("~"){
    ROS_INFO("Speed Command Generator Initialized");
    //if(!node_handle.getParam("topic_name",odom_topic_name)){
   //     ROS_WARN("Did not receive topic_name param. Using /commands/motor/speed");
    //    odom_topic_name = "/odom";
    //}

    //odom_sub = node_handle.subscribe(odom_topic_name,10,&SpeedCommandGeneratorBase::odom_callback,this);
}

SpeedCommandGeneratorBase::SpeedCommandGeneratorBase(ros::NodeHandle &n) : SpeedCommandGeneratorBase(){
    node_handle = n;
}

SpeedCommandGeneratorBase &SpeedCommandGeneratorBase::set_limits(double m_acc,double max_speed,double min_speed){
    this->max_acc_limit = m_acc;
    this->max_linear_speed = max_speed;
    this->min_linear_speed = min_speed;

    return *this;
}

/*SpeedCommandGeneratorBase &SpeedCommandGeneratorBase::set_keep_sending_messages(bool send){
    this->keepSendingMessages = send;
    
    if(send){
        message_timer = node_handle.createTimer(message_sending_rate,&SpeedCommandGeneratorBase::send_msg,this);
    }else{
        message_timer.stop();
    }

    return *this;
}

SpeedCommandGeneratorBase &SpeedCommandGeneratorBase::set_message_rate(ros::Duration rate){
    this->message_sending_rate = rate;
    
    message_timer.setPeriod(rate);

    return *this;
}*/