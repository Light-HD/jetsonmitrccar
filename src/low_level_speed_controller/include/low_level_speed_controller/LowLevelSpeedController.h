#pragma once

#include <type_traits>
#include <memory>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

#include "low_level_speed_controller/SpeedCommandInterfaceBase.h"
#include "low_level_speed_controller/SpeedCommandGeneratorBase.h"

#include "math_utils.h"

class SpeedCommandInterfaceBase;
class SpeedCommandGeneratorBase;


//template<class TCommandGen, class TSpeedInterface>
class LowLevelSpeedController{
    public:
        //static_assert(std::is_base_of<SpeedCommandInterfaceBase, TSpeedInterface>::value,"Class should inherit SpeedCommandInterface");
        //static_assert(std::is_base_of<SpeedCommandGeneratorBase, TCommandGen>::value,"Class Should inherit SpeedCommandInterface");
        
        enum ControlMsgType { Twist, Ackermann, AckermannStamped };
        enum ControlType { Speed, Acceleration };
        //enum OperationMode { Manual, Automatic };

           
        LowLevelSpeedController();
        LowLevelSpeedController(ros::NodeHandle &n);
        LowLevelSpeedController(SpeedCommandGeneratorBase *com_gen, SpeedCommandInterfaceBase *speed_interface);
        LowLevelSpeedController &set_control_msg_type(ControlMsgType ctrl_type);
        LowLevelSpeedController &set_speed_generator(SpeedCommandGeneratorBase *gen);
        LowLevelSpeedController &set_speed_interface(SpeedCommandInterfaceBase *interface);
        LowLevelSpeedController &set_max_limits(double acc, double max_speed, double min_speed);
        LowLevelSpeedController &set_control_type(ControlType type);
        LowLevelSpeedController &set_continously_send_msg(bool send);
        LowLevelSpeedController &set_message_send_rate(ros::Duration duration);
        LowLevelSpeedController &send_direct_speed(double speed_value);
        LowLevelSpeedController &set_goal_point_control_rate(ros::Duration new_rate);
        LowLevelSpeedController &set_goal_point(geometry_msgs::Point &goal_point);

        LowLevelSpeedController &set_use_odom_for_position(bool use);

        LowLevelSpeedController &set_current_position(geometry_msgs::Point &pos);
        
        LowLevelSpeedController &set_use_odom_for_speed(bool use){
            use_odom_for_current_speed = use;

            return *this;
        }
        
        LowLevelSpeedController &set_current_speed(double speed){
            if(!use_odom_for_current_speed)
                return *this;

            current_speed = speed;
            speedInterface->set_current_speed(speed);
            commandGenerator->set_current_speed(speed);
            return *this;
        }

    protected:
        void send_msg(const ros::TimerEvent &e);

        void point_cb(const geometry_msgs::Point::ConstPtr &msg);

        void goal_point_callback(const ros::TimerEvent &e);

        virtual void odom_callback(const nav_msgs::Odometry::ConstPtr &msg){ 
            last_odom_data = *msg;
            
            if(use_odom_for_current_speed){
                current_speed = last_odom_data.twist.twist.linear.x;
                commandGenerator->set_current_speed(current_speed);
                speedInterface->set_current_speed(current_speed);
                commandGenerator->set_odom_data(last_odom_data);
            }

            if(use_odom_for_position){
                current_position = msg->pose.pose.position;
                commandGenerator->set_last_point(current_position);
                commandGenerator->set_odom_data(last_odom_data);
            }

        }
        ros::NodeHandle node_handle;
        ros::Subscriber command_sub;
        ros::Subscriber point_sub;
        ros::Subscriber odom_sub;

        ControlMsgType control_msg_type;
        ControlType control_type;
        
        SpeedCommandGeneratorBase *commandGenerator;
        SpeedCommandInterfaceBase *speedInterface;
        
        double current_speed;
        double calculated_command;
        
        SpeedCommandInterfaceBase::CommandRequest motor_cmd;
        
        ros::Timer message_timer;

        ros::Timer goal_point_callback_timer;
        ros::Duration goal_point_control_rate;

        std::string odom_topic_name;
        std::string point_topic_name;
        std::string control_topic_name;

        ros::Duration message_rate;
        bool keepSendingCommands;

        bool use_odom_for_current_speed;
        nav_msgs::Odometry last_odom_data;

        geometry_msgs::Twist last_msg_twist;
        ackermann_msgs::AckermannDrive last_msg_ackermann;
        ackermann_msgs::AckermannDriveStamped last_msg_ackermann_stamped;

        bool use_odom_for_position;
        geometry_msgs::Point current_position;

        geometry_msgs::Point goal_point;
        bool goal_still_active;

    private:
        void twist_callback(const geometry_msgs::Twist::ConstPtr &msg);
        void ackermann_callback(const ackermann_msgs::AckermannDrive::ConstPtr &msg);
        void ackermann_stamped_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg);
        
};
