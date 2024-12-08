//
// Created by qyd on 24-12-6.
//
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
class OdomPub
{
  public:
    OdomPub()
    {
      odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
      tf_broadcaster = new tf2_ros::TransformBroadcaster();
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "base_link";
      ros::Time last_time =ros::Time::now();

    }
    void updateOdometry(double vr_LF , double vr_LB , double vr_RF , double vr_RB, double r)
    {
      ros::Time current_time = ros::Time::now();
      double dt = (current_time - last_time).toSec();
      double vx = (r /4.0) * (vr_LF + vr_RF+ vr_LB + vr_RB);
      double vy = (r /4.0) * (vr_LF - vr_RF - vr_LB + vr_RB);
      double vtheta = (r /4.0) * (vr_LF + vr_RF - vr_LB - vr_RB);
      double delta_x = vx * dt;
      double delta_y = vy * dt;
      double delta_theta = vtheta * dt;
      x += delta_x;
      y += delta_y;
      theta += delta_theta;
      odom_msg.header.stamp = current_time;
      odom_msg.pose.pose.position.x = x;
      odom_msg.pose.pose.position.y = y;
      odom_msg.pose.pose.position.z = 0;
      tf2::Quaternion q;
      q.setRPY(0, 0, theta);
      odom_msg.pose.pose.orientation.x = q.x();
      odom_msg.pose.pose.orientation.y = q.y();
      odom_msg.pose.pose.orientation.z = q.z();
      odom_msg.pose.pose.orientation.w = q.w();
      odom_pub.publish(odom_msg);
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.child_frame_id = "base_link";
      odom_trans.header.frame_id = "odom";
      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_msg.pose.pose.orientation; ;
      tf_broadcaster->sendTransform(odom_trans);
   }
 private:
    ros::NodeHandle nh;
    ros::Publisher odom_pub;
    tf2_ros::TransformBroadcaster *tf_broadcaster;
    nav_msgs::Odometry odom_msg;

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    ros::Time last_time ;
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_pub");
  OdomPub odom_pub;
  ros::Rate loop_rate(50);
  double r = 0.4;
  double vr_LF = 0.0, vr_LB = 0.0, vr_RF = 0.0, vr_RB = 0.0;
  while (ros::ok())
    {
    odom_pub.updateOdometry(vr_LF,vr_LB,vr_RF,vr_RB,r);
    loop_rate.sleep();
    }
}