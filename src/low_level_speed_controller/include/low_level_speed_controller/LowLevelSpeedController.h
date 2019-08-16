#pragma once

#include <type_traits>
#include <memory>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

#include "low_level_speed_controller/SpeedCommandInterfaceBase.h"
#include "low_level_speed_controller/SpeedCommandGeneratorBase.h"


class SpeedCommandInterfaceBase;
class SpeedCommandGeneratorBase;


template<class TCommandGen, class TSpeedInterface>
class LowLevelSpeedController{
    public:
        static_assert(std::is_base_of<SpeedCommandInterfaceBase, TSpeedInterface>::value,"Class should inherit SpeedCommandInterface");
        static_assert(std::is_base_of<SpeedCommandGeneratorBase, TCommandGen>::value,"Class Should inherit SpeedCommandInterface");
        
        enum ControlMsgType { Twist, Ackermann, AckermannStamped };
        enum ControlType { Speed, Acceleration };
        //enum OperationMode { Manual, Automatic };

           
        LowLevelSpeedController();
        LowLevelSpeedController(ros::NodeHandle &n);
        LowLevelSpeedController(std::shared_ptr<SpeedCommandGeneratorBase> com_gen, 
                                    std::shared_ptr<SpeedCommandInterfaceBase> speed_interface);
        LowLevelSpeedController(std::unique_ptr<TCommandGen> com_gen, 
        std::unique_ptr<TSpeedInterface> s_int);
        LowLevelSpeedController &set_control_msg_type(ControlMsgType ctrl_type);
        LowLevelSpeedController &set_speed_generator(std::shared_ptr<SpeedCommandGeneratorBase> gen);
        LowLevelSpeedController &set_speed_interface(std::shared_ptr<SpeedCommandInterfaceBase> interface);
        LowLevelSpeedController &set_max_limits(double acc, double max_speed, double min_speed);
        LowLevelSpeedController &set_control_type(ControlType type);
        LowLevelSpeedController &set_continously_send_msg(bool send);
        LowLevelSpeedController &set_message_send_rate(ros::Duration duration);

        const TSpeedInterface *get_speed_interface() const { return speedInterface.get(); }
        const TCommandGen *get_speed_interface() const { return commandGenerator.get(); }
        
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

        virtual void odom_callback(nav_msgs::Odometry::ConstPtr &msg){ 
            last_odom_data = *msg; 
            
            if(use_odom_for_current_speed){
                current_speed = last_odom_data.twist.twist.linear.x;
                commandGenerator->set_current_speed(current_speed);
                speedInterface->set_current_speed(current_speed);
            }
        }
        ros::NodeHandle node_handle;
        ros::Subscriber command_sub;

        ControlMsgType control_msg_type;
        ControlType control_type;
        
        std::unique_ptr<SpeedCommandGeneratorBase> commandGenerator;
        std::unique_ptr<SpeedCommandInterfaceBase> speedInterface;
        
        double current_speed;
        double calculated_command;
        
        SpeedCommandInterfaceBase::CommandRequest motor_cmd;
        
        ros::Timer message_timer;
        std::string control_topic_name;

        ros::Duration message_rate;
        bool keepSendingCommands;

        bool use_odom_for_current_speed;
        nav_msgs::Odometry last_odom_data;

        geometry_msgs::Twist last_msg_twist;
        ackermann_msgs::AckermannDrive last_msg_ackermann;
        ackermann_msgs::AckermannDriveStamped last_msg_ackermann_stamped;

    private:
        void twist_callback(const geometry_msgs::Twist::ConstPtr &msg);
        void ackermann_callback(const ackermann_msgs::AckermannDrive::ConstPtr &msg);
        void ackermann_stamped_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg);
        
};
