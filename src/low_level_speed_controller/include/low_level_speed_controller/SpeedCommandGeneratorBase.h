#pragma once

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "low_level_speed_controller/SpeedCommandInterfaceBase.h"
#include "math_utils.h"

typedef SpeedCommandInterfaceBase::CommandRequest CommandRequest;

class SpeedCommandGeneratorBase{
    public:
        SpeedCommandGeneratorBase();
        SpeedCommandGeneratorBase(ros::NodeHandle &n);

        virtual ~SpeedCommandGeneratorBase(){}

        enum ControlType{Speed, Acceleration};

        SpeedCommandGeneratorBase &set_limits(double acc, double max, double min);

        SpeedCommandGeneratorBase &set_control_type(ControlType type){
            control_type = type;
            
            return *this;
        }

        SpeedCommandGeneratorBase &set_odom_data(nav_msgs::Odometry msg){
            last_odom_msg = msg;
            return *this;
        }

        SpeedCommandGeneratorBase &set_current_speed(double speed){
            current_speed = speed;

            return *this;
        }

        //SpeedCommandGeneratorBase &set_keep_sending_messages(bool);
        //SpeedCommandGeneratorBase &set_message_rate(ros::Duration);

        double get_max_acceleration() const { return max_acc_limit; }
        double get_min_speed() const { return min_linear_speed; }
        double get_max_linear_speed() const { return max_linear_speed; }

        virtual CommandRequest createSpeedCommand(double speed_setpoint) = 0;

    private:
    protected:
        
        //virtual void odom_callback(const nav_msgs::Odometry::ConstPtr &msg){ last_odom_msg = *msg; }
        //void send_msg(ros::TimerEvent &e);
        std::string odom_topic_name;
        CommandRequest last_request;
        ControlType control_type;

        ros::NodeHandle node_handle;
        double max_acc_limit;
        double max_linear_speed;
        double min_linear_speed;
        //bool keepSendingMessages;
        double current_speed;
        //ros::Duration message_sending_rate;
        nav_msgs::Odometry last_odom_msg;
        //ros::Subscriber odom_sub;
        //ros::Timer message_timer;
};