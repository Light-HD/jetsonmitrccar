#pragma once
#include <queue>
#include <ros/ros.h>
#include "math_utils.h"

#define DEBUG

class SpeedCommandInterfaceBase{
    public:
        SpeedCommandInterfaceBase();
        SpeedCommandInterfaceBase(ros::NodeHandle &n);

        virtual ~SpeedCommandInterfaceBase(){}

        enum OperationType{ AUTOMATIC, MANUAL };

        struct CommandRequest{
            enum RequestType{ SPEED, BRAKE, TAKEOFF, ACCELERATION };
            double value;
            RequestType req_type;
            ros::Time req_time;
        };

        void queue_command(struct CommandRequest &command){ 
            //ROS_INFO("COMMAND RECEIVED"); 
            command_queue.push(command); 
        }
        int return_remaining_command_count() const{ return command_queue.size(); }

        SpeedCommandInterfaceBase &set_max_limits(double acc,double max_speed,double min_speed);
        SpeedCommandInterfaceBase &set_operation_type(OperationType);
        SpeedCommandInterfaceBase &set_command_timeout(ros::Duration);
        SpeedCommandInterfaceBase &set_execution_rate(ros::Duration);
        SpeedCommandInterfaceBase &set_current_speed(double);

        void issue_write_command(){
            if(op_type == OperationType::AUTOMATIC){
                return;
            }

            if(command_queue.size()){
                CommandRequest comm = command_queue.front();
                
                if(!check_command_integrity(comm)){
                    return;
                }

                send_command(comm, true);
                command_queue.pop();
                //ROS_INFO("COMMAND PROCESSED");
            }else{
                ROS_WARN("NO COMMAND LEFT TO SEND");
            }
        }

        const OperationType &get_operation_type() const { return op_type; }
        const ros::Duration &get_command_timeout() const { return command_timeout; }
        double get_max_acceleration() const { return max_acceleration; }
        double get_max_speed() const { return max_speed; }
        double get_min_speed() const { return min_speed; }

    private:
    protected:
        void automatic_callback(const ros::TimerEvent &event){
             if(command_queue.size()){
                CommandRequest comm = command_queue.front();

                if(!check_command_integrity(comm)){
                    return;
                }

                send_command(comm, false);
                command_queue.pop();
                //ROS_INFO("COMMAND PROCESSED");
            }else{
                ROS_WARN("NO COMMAND LEFT TO SEND");
            }
        }
        virtual bool send_command(struct CommandRequest &command, bool manual = false) = 0;
        double max_acceleration;
        double max_speed;
        double min_speed;
        double current_speed;
        ros::Duration command_timeout;
        ros::Duration auto_execute_rate;
        std::queue<struct CommandRequest> command_queue;
        OperationType op_type;
        ros::NodeHandle node_handle;
        ros::Timer callback_timer;

        

        virtual bool check_command_integrity(CommandRequest &command){
            if(abs(command.value) > max_speed){
                ROS_ERROR("SPEED COMMAND IS LARGER THAN MAXIMUM VALUE %f", command.value);
                return false;
            }

            if(abs(command.value) < min_speed){
                ROS_ERROR("SPEED COMMAND IS SMALLER THAN MINIMUM VALUE %f", command.value);
                return false;
            }

            if(abs(command.value - current_speed) > max_acceleration){
                ROS_ERROR("SPEED DIFFERENCE IS LARGER THAN MAX ACCELERATION");
                return false;
            }

            if(!check_command_timestamp(command)){
                ROS_ERROR("COMMAND HAS TIMED OUT");
                return false;
            }


            return true;
        }

        inline bool check_command_timestamp(CommandRequest &x){
            return (ros::Time::now() - x.req_time) < command_timeout;
        }
};
