#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

class VelocityController {
public:
    VelocityController()
        : nh("~"), tf_listener(tf_buffer) {
        cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &VelocityController::cmdVelCallback, this);
        nh.param("wheel_base", wheel_base, 0.4);
        nh.param("velocity_mode", velocity_mode, std::string("odom"));
    }

    void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg) {
        double vx = msg->linear.x;
        double vy = msg->linear.y;
        double wz = msg->angular.z;
        double v1, v2, v3, v4;

        if (velocity_mode == "base_link") {
            inverseKinematics(vx, vy, wz, v1, v2, v3, v4);
            excuteWheelSpeeds(v1, v2, v3, v4);
        } else if (velocity_mode == "odom") {
            geometry_msgs::Twist transformed_cmd_vel;
            if (transformVelocityToBaseLink(msg, transformed_cmd_vel)) {
                inverseKinematics(transformed_cmd_vel.linear.x, transformed_cmd_vel.linear.y,
                                   transformed_cmd_vel.angular.z, v1, v2, v3, v4);
                excuteWheelSpeeds(v1, v2, v3, v4);
            } else {
                ROS_WARN("Failed to transform velocity to base_link frame");
            }
        } else {
            ROS_WARN("Unknown velocity mode");
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber cmd_vel_sub;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    double wheel_base;
    std::string velocity_mode;

    void inverseKinematics(double vx, double vy, double wz, double& v1, double& v2, double& v3, double& v4) {
        v1 = vx - vy - (wheel_base * wz);
        v2 = vx + vy + (wheel_base * wz);
        v3 = vx + vy - (wheel_base * wz);
        v4 = vx - vy + (wheel_base * wz);  // 错误修正：应该使用 wz 代替 vz
    }

    void excuteWheelSpeeds(double v1, double v2, double v3, double v4) {
        ROS_INFO("v1: %f, v2: %f, v3: %f, v4: %f", v1, v2, v3, v4);
    }

    bool transformVelocityToBaseLink(const geometry_msgs::TwistConstPtr& cmd_vel, geometry_msgs::Twist& transformed_cmd_vel) {
        try {
            geometry_msgs::TransformStamped transform_stamped;
            transform_stamped = tf_buffer.lookupTransform("base_link", "odom", ros::Time(0));
            tf2::doTransform(*cmd_vel, transformed_cmd_vel, transform_stamped);
            return true;
        } catch (const tf2::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            return false;  // 返回 false 表示转换失败
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_transform");
    VelocityController velocity_controller;
    ros::spin();
    return 0;
}

