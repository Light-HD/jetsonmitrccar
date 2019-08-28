#include "ros/ros.h"

#include "low_level_speed_controller/LowLevelSpeedController.h"
#include "low_level_speed_controller/SixWheelSpeedInterface.h"
#include "low_level_speed_controller/SixWheelSpeedGenerator.h"



LowLevelSpeedController *test;




int main(int argc,char **argv){
    ros::init(argc, argv, "six_wheel_low_level_speed_controller");
    ros::NodeHandle n("~");
    
    SixWheelSpeedInterface SixWheel(n);
    SixWheelSpeedGenerator SixWheel_speed_gen(n);
    SixWheel.set_command_timeout(ros::Duration(1.0));
    SixWheel.set_operation_type(SpeedCommandInterfaceBase::AUTOMATIC);
    SixWheel.set_execution_rate(ros::Duration(1.0 / 50.0));


    SixWheel_speed_gen.set_limits(1000.0, 1.0, 0.26);
    SixWheel.set_max_limits(10000.0, 3000.0, 0.0);
    SixWheel.set_speed_offset(0.215367751);
    SixWheel.set_speed_gain(0.002429281);
    
    
    test = new LowLevelSpeedController(n);

    test->set_speed_interface(&SixWheel).set_speed_generator(&SixWheel_speed_gen)
                .set_control_msg_type(LowLevelSpeedController::ControlMsgType::Twist)
                    .set_control_type(LowLevelSpeedController::ControlType::Speed)
                        .set_continously_send_msg(true)
                            .set_message_send_rate(ros::Duration(1.0 / 50.0))
                                .set_use_odom_for_speed(true).set_use_odom_for_position(true);

    ros::spin();
    return 0;
}