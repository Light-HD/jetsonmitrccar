#include <ros/ros.h>
#include <rc_msgs/RCControlMsg.h>
#include <std_msgs/Float64.h>
#include <vesc_msgs/VescStateStamped.h>

#define ACC_CONTROL 1

//#define SPEED_CONTROL 1
#define DEBUG 1
//#define SENSORLESS_DEBUG 1

#define Max_Speed_Change 1000
#define Max_Servo_Change 0.05
/* ROS Package to publish ackerrman msgs according to vesc controllers ROS topic structure.
   Vesc receives data from to seperate topics published as std_msgs::Float64
 */

// 0.5 is the center for servo
//

inline double max_double(double a,double b){
    return (a > b) ? (a) : (b);
}

inline double min_double(double a,double b){
    return (a > b) ? (b) : (a);
}

inline double sgn(double a){
    if(a > 0.0){
        return 1.0;
    }

    return -1.0;
}

inline bool near(double a, double b,double distance){
    return ((a - b) * sgn(a - b) < distance);
}

inline double abs_double(double a){
    return a * sgn(a);
}

class RC_Driver_vesc{
public:

    RC_Driver_vesc(){
        n = ros::NodeHandle("~");
        ROS_INFO("INITIALIZING");

        if(n.getParam("max_throttle_output",max_throttle_output) && n.getParam("max_steering_output",max_steering_output) 
        && n.getParam("min_throttle_output",min_throttle_output) && n.getParam("min_steering_output",min_steering_output) 
        && n.getParam("middle_throttle_output",middle_throttle_output) && n.getParam("middle_steering_output",middle_steering_output) 
        && n.getParam("max_car_linear_speed",max_car_linear_speed) && n.getParam("max_car_angle",max_car_angle)
        && n.getParam("min_car_linear_speed",min_car_linear_speed) && n.getParam("min_car_angle",min_car_angle)
        && n.getParam("max_linear_acc",max_linear_acc) && n.getParam("max_angular_acc",max_angular_acc)
        && n.getParam("motor_alpha",motor_alpha) && n.getParam("servo_alpha",servo_alpha)){
            
        }else{
            ROS_ERROR("THERE ARE MISSING PARAMETERS PLEASE SUPPLY REQUIRED PARAMETERS. EXITING...");
            return;
        }

        motor_topic = std::string("/commands/motor/speed");
        servo_topic = std::string("/commands/servo/position");

        accumulated_motor_value = 0.0;
        accumulated_servo_value = 0.0;

        current_motor_speed = min_car_linear_speed;
        current_servo_pos = 0.0;

        n.getParam("motor_topic",motor_topic);
        n.getParam("servo_topic",servo_topic);
        n.getParam("rc_topic",rc_command_topic);
        n.getParam("sensor_topic",sensor_topic);

        rc_command_sub = n.subscribe(rc_command_topic,10,&RC_Driver_vesc::rc_command_callback_unique_command,this);
        sensor_subs = n.subscribe(sensor_topic,10,&RC_Driver_vesc::sensor_callback,this);

        vesc_motor_pub = n.advertise<std_msgs::Float64>(motor_topic,10);
        vesc_servo_pub = n.advertise<std_msgs::Float64>(servo_topic,10);
        _timer = n.createTimer(ros::Duration(1.0/50.0), &RC_Driver_vesc::timerCallback, this);
        //TODO Implement non unique_command
    }


    void timerCallback(const ros::TimerEvent& event){
        vesc_motor_pub.publish(motor_msg);
        vesc_servo_pub.publish(servo_msg);
    }

    void sensor_callback(const vesc_msgs::VescStateStamped::ConstPtr &msg){
        current_motor_speed = msg->state.speed;
    }


    void rc_command_callback_unique_command(const rc_msgs::RCControlMsg::ConstPtr &msg){
        //Messages are only zero if RC is closed. Discard Values;
        if(msg->steering_cmd == 0 || msg->throttle_cmd == 0){
            ROS_WARN("RC IS CLOSED. OPEN RC TO CONTROL");
            return;
        }

        //ROS_INFO("GOT CALLBACK");

        //MIT Racecar Lightweight 2-D Simulator Updates poses on regular intervals.
        //If same command is being published from RC, then there is no need to publish this since
        //Pose still get updated.

        //ROS_INFO("AT CALLBACK");

        double throttle_percentage = 0.0;
        double steer_percentage = 0.0;

        if(msg->steering_cmd > middle_steering_output){
            steer_percentage = (msg->steering_cmd - middle_steering_output)/(max_steering_output - middle_steering_output);
        }else{
            steer_percentage = -(middle_steering_output - msg->steering_cmd)/(middle_steering_output - min_steering_output);
        }

        if(msg->throttle_cmd > middle_throttle_output){
            throttle_percentage = (msg->throttle_cmd - middle_throttle_output)/(max_throttle_output - middle_throttle_output);
        }else{
            throttle_percentage = -(middle_throttle_output - msg->throttle_cmd)/(middle_throttle_output - min_throttle_output);
        }

        
        if(steer_percentage > 1.0){
            steer_percentage = 1.0;
        }

        if(steer_percentage < -1.0){
            steer_percentage = -1.0;
        }

        if(throttle_percentage > 1.0){
            throttle_percentage = 1.0;
        }

        if(throttle_percentage < -1.0){
            throttle_percentage = -1.0;
        }

        accumulated_motor_value = (motor_alpha * throttle_percentage) + ((1 - motor_alpha) * accumulated_motor_value);
        accumulated_servo_value = (servo_alpha * steer_percentage) + ((1 - servo_alpha) * accumulated_servo_value);

        #ifdef SPEED_CONTROL
        float throttle_interval = max_car_linear_speed - min_car_linear_speed;
        float steering_interval = max_car_angle - min_car_angle;

        float speed_output = sgn(throttle_percentage) * (min_car_linear_speed + (sgn(throttle_percentage) * throttle_interval * throttle_percentage));
        float steer_output = sgn(steer_percentage) * (min_car_angle + (sgn(steer_percentage) * steering_interval) * steer_percentage);
            
            
        if(sgn(steer_percentage) * steer_percentage < 0.07){
            steer_output = 0;
        }

        if(sgn(throttle_percentage) * throttle_percentage < 0.07){
            speed_output = 0;
        }

        motor_msg.data = speed_output;
        servo_msg.data = (steer_output / 2.0) + 0.5;
        #endif


        #ifdef ACC_CONTROL

        #ifdef DEBUG
            ROS_INFO("Accumulated Motor Value %f Accumulated Servo Value %f",accumulated_motor_value,accumulated_servo_value);
        #endif

        double motor_cmd;

        if(accumulated_motor_value > -0.2 && accumulated_motor_value < 0.0){
            #ifdef DEBUG
                ROS_INFO("We are in speed control region of ACC_CONTROL");
            #endif
            double throttle_interval = max_car_linear_speed - min_car_linear_speed;
            motor_cmd = sgn(throttle_percentage) * (min_car_linear_speed + (abs_double(accumulated_motor_value) * throttle_interval));
        }else{
            if(current_motor_speed > 0.0){
                motor_cmd = min_double(max_car_linear_speed,current_motor_speed + (accumulated_motor_value * max_linear_acc));
            }else{
                motor_cmd = max_double(-max_car_linear_speed,current_motor_speed + (accumulated_motor_value * max_linear_acc));
            }
        }


        if(abs_double(accumulated_motor_value) < 0.07){
            motor_cmd = 0.0;
        }

        

        //double servo_cmd;

        // We are Speeding up. Check if We are still inside max linear Speed
        /*if(accumulated_motor_value > 0.0 && current_motor_speed > 550.0){
            #ifdef DEBUG
            ROS_INFO("ACCELERATING FORWARD");
            #endif
            motor_cmd = min_double(max_car_linear_speed,current_motor_speed + (accumulated_motor_value * max_linear_acc));
            motor_cmd = max_double(550.0,motor_cmd);
        }
        // We are Slowing Down going backwards. Currently not jumping to positive min_car_linear_speed
        else if(accumulated_motor_value > 0.0 && current_motor_speed < 550.0){
            #ifdef DEBUG
            ROS_INFO("SLOWING DOWN BACKWARDS");
            #endif
            motor_cmd = current_motor_speed + (accumulated_motor_value * max_linear_acc);
        }

        // We are accelerating at backwards. Check for max limits
        else if(accumulated_motor_value < 0.0 && current_motor_speed < 550.0){
            #ifdef DEBUG
            ROS_INFO("ACCELERATING BACKWARDS");
            #endif
            motor_cmd = max_double(-max_car_linear_speed,current_motor_speed + (accumulated_motor_value * max_linear_acc));
            motor_cmd = min_double(-550.0, motor_cmd);
        }

        // We are slowing down going forward. Currently not jumping to -min_car_linear_speed
        else if(accumulated_motor_value < 0.0 && current_motor_speed > 550.0){
            #ifdef DEBUG
            ROS_INFO("SLOWING DOWN FORWARD");
            #endif
            motor_cmd = current_motor_speed + (accumulated_motor_value * max_linear_acc);
        }

        if(abs_double(accumulated_motor_value) < 0.1){
            #ifdef DEBUG
            ROS_INFO("MIDDLE. SLOWING DOWN");
            #endif
            motor_cmd = (current_motor_speed > 0.0) ? (-max_linear_acc) : (max_linear_acc);
            motor_cmd = (abs_double(current_motor_speed) < 1300.0) ? (0.0) : (motor_cmd);
        }*/

        motor_msg.data = motor_cmd;

        #ifdef SENSORLESS_DEBUG
        current_motor_speed = motor_cmd;
        #endif

        //motor_msg.data = (accumulated_servo_value * (max_car_linear_speed - min_car_linear_speed)) + (sgn(accumulated_motor_value) * min_car_linear_speed);
        servo_msg.data = (accumulated_servo_value / 2.0) + 0.5;

        #ifdef DEBUG
        ROS_INFO("SETPOINT FOR MOTOR SPEED %f",motor_cmd);
        #endif

        #endif
    }

private:
    ros::Subscriber rc_command_sub;
    ros::Subscriber sensor_subs;
    ros::Publisher vesc_motor_pub;
    ros::Publisher vesc_servo_pub;
    
    
    ros::Timer _timer;
    ros::NodeHandle n;
    //seq_count for drive message stamp
    
    std_msgs::Float64 motor_msg;
    std_msgs::Float64 servo_msg;
    //Store old Command Value
    rc_msgs::RCControlMsg command_value;
    
    double accumulated_motor_value;
    double accumulated_servo_value;

    double current_motor_speed;
    double current_servo_pos;

    double motor_alpha;
    double servo_alpha;

    double max_linear_acc;
    double max_angular_acc;

    //Store Max and Min RC Values and the middle point
    double max_throttle_output;
    double max_steering_output;

    double min_throttle_output;
    double min_steering_output;

    double middle_throttle_output;
    double middle_steering_output;

    //Store Max and Min Values of the Car.
    //Output will be based on percentage value and these values is
    //necessary to output correct values;

    double max_car_linear_speed;
    double max_car_angle;

    double min_car_linear_speed;
    double min_car_angle;

    //std::string drive_topic;
    std::string rc_command_topic;
    std::string sensor_topic;
    std::string motor_topic;
    std::string servo_topic;
};




int main(int argc,char **argv){
    ros::init(argc,argv,"ackermann_to_vesc_cmd");
    RC_Driver_vesc vesc;
    ros::spin();
    
    return 0;
}

