#include "low_level_speed_controller/VescSpeedInterface.h"
#include "std_msgs/Float64.h"

VescSpeedInterface::VescSpeedInterface() : SpeedCommandInterfaceBase(){
    if(!node_handle.getParam("topic_name",topic_name)){
        ROS_WARN("Did not receive topic_name param. Using /commands/motor/speed");
        topic_name = "/commands/motor/speed";
    }

    data_publisher = node_handle.advertise<std_msgs::Float64>(topic_name,10,false);
}

VescSpeedInterface::VescSpeedInterface(ros::NodeHandle &n) : SpeedCommandInterfaceBase(n){
    if(!node_handle.getParam("topic_name",topic_name)){
        ROS_WARN("Did not receive topic_name param. Using /commands/motor/speed");
        topic_name = "/commands/motor/speed";
    }

    data_publisher = node_handle.advertise<std_msgs::Float64>(topic_name,10,false);
}


bool VescSpeedInterface::send_command(CommandRequest &comm, bool manual = false){
    std_msgs::Float64 msg;
    msg.data = comm.value;

    data_publisher.publish(msg);
}