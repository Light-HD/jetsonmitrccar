
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <tgmath.h>
#include "std_msgs/Float64.h"




class Q2YAW
{
 public:

    Q2YAW ()
    {
      ros::NodeHandle n;
     sub = n.subscribe("/imu/data", 1000,&Q2YAW::imuCallback,this);
     yaw_pub = n.advertise<std_msgs::Float64>("/yaw", 1000, true);


    }


    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {

      double siny_cosp = +2.0 * (msg->orientation.w * msg->orientation.z + msg->orientation.x * msg->orientation.y);
      double cosy_cosp = +1.0 - 2.0 * (msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z);
      double yaw = atan2(siny_cosp, cosy_cosp);

      yaw_angle.data=yaw*57;

    yaw_pub.publish(yaw_angle);
    }


  private:
  ros::Subscriber sub;
  ros::Publisher yaw_pub;
  std_msgs::Float64 yaw_angle;



};






int main(int argc, char **argv)
{
  ros::init(argc, argv, "quat_to_yaw");

  Q2YAW obj;

  ros::spin();
  return 0;
}

