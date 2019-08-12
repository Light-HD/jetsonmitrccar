#include "low_level_speed_controller/LowLevelSpeedController.h"
#include "low_level_speed_controller/VescSpeedGenerator.h"
#include "low_level_speed_controller/VescSpeedInterface.h"

LowLevelSpeedController ::LowLevelSpeedController() : node_handle("~"), current_speed(0), message_rate(0.1)
{
    ROS_INFO("Low Level Speed Controller Initialized");
    control_topic_name = "/cmd_vel";
    control_msg_type = ControlMsgType::Twist;
    control_type = ControlType::Speed;
}

LowLevelSpeedController ::LowLevelSpeedController(ros::NodeHandle &n) : LowLevelSpeedController()
{
    node_handle = n;

    if (!node_handle.getParam("control_topic_name", control_topic_name))
    {
        ROS_WARN("COULDNT FIND PARAM control_topic_name. USING /cmd_vel");
    }
}

LowLevelSpeedController ::
    LowLevelSpeedController(SpeedCommandGeneratorBase *com_gen, SpeedCommandInterfaceBase *s_int)
    : LowLevelSpeedController()
{

    //SpeedCommandGeneratorBase *command_base = static_cast<SpeedCommandGeneratorBase>(com_gen);
    //SpeedCommandInterfaceBase *s_interface = static_cast<SpeedCommandInterfaceBase>(s_int);

    commandGenerator = com_gen;
    speedInterface = s_int;
    //speedInterface->set_operation_type(SpeedCommandInterfaceBase::OperationType::MANUAL);
}

LowLevelSpeedController &LowLevelSpeedController::set_control_msg_type(ControlMsgType ctrl_type)
{

    this->control_msg_type = ctrl_type;

    switch (ctrl_type)
    {
    case ControlMsgType::Twist:
        command_sub = node_handle.subscribe(control_topic_name, 10,
                                            &LowLevelSpeedController ::twist_callback, this);
        break;

    case ControlMsgType::Ackermann:
        command_sub = node_handle.subscribe(control_topic_name, 10,
                                            &LowLevelSpeedController ::ackermann_callback, this);
        break;

    case ControlMsgType::AckermannStamped:
        command_sub = node_handle.subscribe(control_topic_name, 10,
                                            &LowLevelSpeedController ::ackermann_stamped_callback, this);
        break;
    }

    return *this;
}

LowLevelSpeedController &LowLevelSpeedController ::set_control_type(ControlType ctrl_type)
{

    this->control_type = ctrl_type;

    switch (ctrl_type)
    {
    case ControlType::Acceleration:
        commandGenerator->set_control_type(SpeedCommandGeneratorBase::ControlType::Acceleration);
        break;

    case ControlType::Speed:
        commandGenerator->set_control_type(SpeedCommandGeneratorBase::ControlType::Speed);
        break;
    }

    return *this;
}

void LowLevelSpeedController::twist_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    last_msg_twist = *msg;
    motor_cmd = commandGenerator->createSpeedCommand(last_msg_twist.linear.x);

    if (!keepSendingCommands)
    {
        motor_cmd.req_time = ros::Time::now();
        speedInterface->queue_command(motor_cmd);
    }
}

void LowLevelSpeedController::send_msg(const ros::TimerEvent &e)
{
    motor_cmd.req_time = ros::Time::now();
    speedInterface->queue_command(motor_cmd);
}

LowLevelSpeedController &LowLevelSpeedController::set_speed_generator(SpeedCommandGeneratorBase *gen)
{
    commandGenerator = gen;
}

LowLevelSpeedController &LowLevelSpeedController::set_message_send_rate(ros::Duration dur)
{
    message_rate = dur;
    message_timer.setPeriod(dur);
}

LowLevelSpeedController &LowLevelSpeedController::set_speed_interface(SpeedCommandInterfaceBase *base)
{
    speedInterface = base;
}

LowLevelSpeedController &LowLevelSpeedController ::set_continously_send_msg(bool send)
{
    keepSendingCommands = send;

    switch (send)
    {
    case true:
        message_timer = node_handle.createTimer(message_rate,
                                                &LowLevelSpeedController::send_msg, this);
        break;

    case false:
        message_timer.stop();
        break;
    }
}

void LowLevelSpeedController::ackermann_callback(const ackermann_msgs::AckermannDrive::ConstPtr &msg){

}

void LowLevelSpeedController::
    ackermann_stamped_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg){
    
}

LowLevelSpeedController &LowLevelSpeedController::set_max_limits(double acc, double max_sp, double min_sp){
    speedInterface->set_max_limits(acc, max_sp, min_sp);
    commandGenerator->set_limits(acc, max_sp,min_sp);
}


//template class LowLevelSpeedController<VescSpeedGenerator,VescSpeedInterface>;