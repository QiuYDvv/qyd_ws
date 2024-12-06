//
// Created by qyd on 24-12-6.
//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
ros::Publisher cmd_vel_pub;
void publishCmdVel ()
{
  geometry_msgs::Twist msg;
  msg.linear.x = 100.0;
  msg.linear.y = 100.0;
  msg.angular.z = 0;
  cmd_vel_pub.publish(msg);
  ROS_INFO("cmd_vel_pub published cmd_vel:linear.x = %f, linear.y = %f, angular.z = %f", msg.linear.x, msg.linear.y, msg.angular.z);

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_vel_pub");
  ros::NodeHandle nh;
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    publishCmdVel();
    loop_rate.sleep();
  }
  return 0;
}