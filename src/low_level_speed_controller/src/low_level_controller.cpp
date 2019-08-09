#include "ros/ros.h"
#include "low_level_speed_controller/LowLevelSpeedController.h"





int main(int argc,char **argv){
    ros::init(argc, argv, "low_level_controller");
    ros::NodeHandle n;

    LowLevelSpeedController<int, SpeedTester> x;

    ros::spinOnce();
    return 0;
}