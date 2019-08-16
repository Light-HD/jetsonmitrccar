#pragma once

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"

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

        SpeedCommandGeneratorBase &set_odom_data(nav_msgs::Odometry &msg){
            last_odom_msg = msg;
            last_position = last_odom_msg.pose.pose.position;

            return *this;
        }

        SpeedCommandGeneratorBase &set_last_point(geometry_msgs::Point &data){
            last_position = data;

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

        virtual void on_goal_reached() = 0;
        virtual void on_goal_initialize() = 0;
        virtual CommandRequest createSpeedCommand(geometry_msgs::Point &goal_point) = 0;
        virtual CommandRequest createSpeedCommand(double speed_setpoint) = 0;

    private:
    protected:
        ros::NodeHandle node_handle;

        CommandRequest last_request;
        
        ControlType control_type;

        std::string odom_topic_name;
        
        double max_acc_limit;
        double max_linear_speed;
        double min_linear_speed;
        
        double current_speed;

        nav_msgs::Odometry last_odom_msg;
        geometry_msgs::Point last_position;

        geometry_msgs::Point goal_point;
};