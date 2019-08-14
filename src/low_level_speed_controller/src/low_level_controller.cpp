#include "ros/ros.h"
#include "low_level_speed_controller/LowLevelSpeedController.h"
#include "low_level_speed_controller/VescSpeedInterface.h"
#include "low_level_speed_controller/VescSpeedGenerator.h"
#include "low_level_speed_controller/SpeedGeneratorFactory.h"

#include "vesc_msgs/VescStateStamped.h"

LowLevelSpeedController *test;

void speed_cb(const vesc_msgs::VescStateStamped::ConstPtr &msg){
    test->set_current_speed(msg->state.speed);
}

int main(int argc,char **argv){
    ros::init(argc, argv, "low_level_controller");
    ros::NodeHandle n;

    //auto gen_ptr = make_unique_speed_generator<VescSpeedGenerator>(n);

    //gen_ptr->set_limits(500.0,0.0,0.0);
    //ROS_INFO("%f",gen_ptr->get_max_acceleration());

    VescSpeedInterface vesc(n);
    VescSpeedGenerator vesc_speed_gen(n);
    vesc.set_command_timeout(ros::Duration(500.0));
    vesc.set_operation_type(SpeedCommandInterfaceBase::AUTOMATIC);
    vesc.set_execution_rate(ros::Duration(1.0 / 20.0));

    //ros::Time last_time = ros::Time::now();

    //while(ros::Time::now() - last_time < ros::Duration(5->0)){
    //    ros::spinOnce();
    //}
    
    test = new LowLevelSpeedController(n);

    test->set_speed_interface(&vesc);
    test->set_speed_generator(&vesc_speed_gen);
    test->set_max_limits(500.0, 4000.0, 0.0);
    test->set_use_odom_for_speed(true);
    test->set_control_msg_type(LowLevelSpeedController::ControlMsgType::Twist);
    test->set_control_type(LowLevelSpeedController::ControlType::Speed);

    
    test->set_control_type(LowLevelSpeedController::ControlType::Acceleration);
    test->set_continously_send_msg(true);
    test->set_message_send_rate(ros::Duration(1.0 / 20.0));

    geometry_msgs::Point origin_point;
    geometry_msgs::Point goal_point;
    goal_point.x = 1.0;
    goal_point.y = 1.0;
    
    test->set_current_speed(1.0);
    test->set_current_position(origin_point);

    test->set_goal_point_control_rate(ros::Duration(0.05));
    test->set_goal_point(goal_point);

    //ROS_INFO("Queue Size %d",vesc.return_remaining_command_count());

    ros::spin();
    return 0;
}