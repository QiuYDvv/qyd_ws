#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

class VelocityController {
public:
  VelocityController() {
    // 初始化订阅器和参数
    cmd_vel_sub =
        nh.subscribe("/cmd_vel", 10, &VelocityController::cmdVelCallback, this);
    nh.param("wheel_base", wheel_base, 0.4);
    nh.param("velocity_mode", velocity_mode, std::string("odom"));
  }

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double wz = msg->angular.z;
    double v1, v2, v3, v4;

    if (velocity_mode == "base_link") {
      // 如果是底盘坐标系速度模式，直接计算
      inverseKinematics(vx, vy, wz, v1, v2, v3, v4);
      excuteWheelSpeeds(v1, v2, v3, v4);
    } else if (velocity_mode == "odom") {
      // 如果是全局坐标系速度模式
      try {
        // 创建一个全局速度向量
        tf::Vector3 vel_world(vx, vy, 0.0); // 世界坐标系中的速度向量

        // 使用 Stamped 包装输入向量，指定时间和输入坐标系
        tf::Stamped<tf::Vector3> vel_world_stamped(vel_world, ros::Time(0),
                                                   "odom");

        tf::Stamped<tf::Vector3> vel_base_stamped; // 用于存储转换后的速度向量

        // 调用 transformVector
        tf_listener.transformVector("base_link", vel_world_stamped,
                                    vel_base_stamped);
        geometry_msgs::Twist transformed_cmd_vel;
        transformed_cmd_vel.linear.x = vel_base_stamped.x();
        transformed_cmd_vel.linear.y = vel_base_stamped.y();
        transformed_cmd_vel.angular.z = wz;

        // 计算轮速并执行
        inverseKinematics(transformed_cmd_vel.linear.x,
                          transformed_cmd_vel.linear.y,
                          transformed_cmd_vel.angular.z, v1, v2, v3, v4);
        excuteWheelSpeeds(v1, v2, v3, v4);

      } catch (tf::TransformException &ex) {
        ROS_WARN("TF exception: %s", ex.what());
        return; // 如果变换失败，跳过本次回调
      }
    } else {
      ROS_WARN("Unknown velocity mode");
    }
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber cmd_vel_sub;
  tf::TransformListener tf_listener; // 使用旧版TF的变换监听器
  double wheel_base;
  std::string velocity_mode;

  // 逆运动学计算函数
  void inverseKinematics(double vx, double vy, double wz, double &v1,
                         double &v2, double &v3, double &v4) {
    v1 = vx - vy - (wheel_base * wz);
    v2 = vx + vy + (wheel_base * wz);
    v3 = vx + vy - (wheel_base * wz);
    v4 = vx - vy + (wheel_base * wz);
  }

  // 执行轮速函数
  void excuteWheelSpeeds(double v1, double v2, double v3, double v4) {
    ROS_INFO("v1: %f, v2: %f, v3: %f, v4: %f", v1, v2, v3, v4);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_transform");
  VelocityController velocity_controller;
  ros::spin();
  return 0;
}
