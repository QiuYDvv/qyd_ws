//
// Created by qyd on 24-12-6.
//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
double wheel_base = 0.4;
double track_width = 0.4;
double wheel_radius = 0.07625;
ros::Publisher left_front_wheel_pub;
ros::Publisher right_front_wheel_pub;
ros::Publisher left_back_wheel_pub;
ros::Publisher right_back_wheel_pub;

void cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double v_x = msg->linear.x;
  double v_y = msg->linear.y;
  double v_theta = msg->angular.z;
  double left_front_wheel_speed = (v_x -v_y -(wheel_base + track_width) *v_theta)/wheel_radius;
  double right_front_wheel_speed = (v_x +v_y +(wheel_base + track_width) *v_theta)/wheel_radius;
  double left_back_wheel_speed = (v_x +v_y -(wheel_base + track_width) *v_theta)/wheel_radius;
  double right_back_wheel_speed = (v_x -v_y +(wheel_base + track_width) *v_theta)/wheel_radius;
  ROS_INFO("Left front wheel speed: %f", left_front_wheel_speed);
  ROS_INFO("Right front wheel speed: %f", right_front_wheel_speed);
  ROS_INFO("Left back wheel speed: %f", left_back_wheel_speed);
  ROS_INFO("Right back wheel speed: %f", right_back_wheel_speed);
  std_msgs::Float64 left_back_wheel_speed_msg;
  std_msgs::Float64 right_back_wheel_speed_msg;
  std_msgs::Float64 left_front_wheel_speed_msg;
  std_msgs::Float64 right_front_wheel_speed_msg;
  left_front_wheel_speed_msg.data = left_front_wheel_speed;
  right_front_wheel_speed_msg.data = right_front_wheel_speed;
  left_back_wheel_speed_msg.data = left_back_wheel_speed;
  right_back_wheel_speed_msg.data = right_back_wheel_speed;
  left_front_wheel_pub.publish(left_front_wheel_speed_msg);
  right_front_wheel_pub.publish(right_front_wheel_speed_msg);
  left_back_wheel_pub.publish(left_back_wheel_speed_msg);
  right_back_wheel_pub.publish(right_back_wheel_speed_msg);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mecanum_wheels_pub");
  ros::NodeHandle n1;
  ros::Subscriber cmd_vel_sub = n1.subscribe("/cmd_vel", 50, cmd_vel_Callback);
  left_front_wheel_pub = n1.advertise<std_msgs::Float64>("/controller/left_front_wheel_controller/command", 50);
  right_front_wheel_pub = n1.advertise<std_msgs::Float64>("/controller/right_front_wheel_controller/command", 50);
  left_back_wheel_pub = n1.advertise<std_msgs::Float64>("/controller/left_back_wheel_controller/command", 50);
  right_back_wheel_pub = n1.advertise<std_msgs::Float64>("/controller/right_back_wheel_controller/command", 50);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}