# My Project
## 每个ros包的说明
### hero_chassis_controller包在config的pid_for_wheels.yaml文件中配置了四个轮子的pid控制器,在src的cmd_vel_pub.cpp编写了一个在/cmd_vel上发布twist消息速度指令的节点，在mecanum_wheels_pub.cpp编写了一个将/cmd_vel上速度指令转换成四个轮子的Float64消息速度指令的每个轮子的速度指令发布在/controller/.../command话题上控制底盘运动，在launch的hero_chassis_controller.launch中加载控制器配置文件和运行控制器管理器和四个控制器
### odom_pub包的odom_pub.cpp文件中实现了里程计消息发布在/odom话题和tf广播base_link和odom坐标转换关系
### tf_tranform包的tf_tranform.cpp实现了将世界坐标系下的/cmd_vel速度指令转换成base_link坐标系下的速度指令并且转换成四个轮子的速度指令控制底盘运动
## 使用命令和效果
```shell
roslaunch hero_chassis_controller hero_chassis_controller.launch 
```
### 启动底盘的gazebo和rviz和加载控制器同时运行odom_pub节点
![launch](https://github.com/QiuYDvv/picture/blob/c7e3e89d47cff99c871c81f6ffd554deede36e08/1.png)
```shell
rosrun hero_chassis_controller cmd_vel_pub
```
### 发布默认的/cmd_vel消息
```shell
rosrun hero_chassis_controller mecanum_wheels_pub
```
### 运行世界坐标系的/cmd_vel速度指令控制底盘运动
```shell
rosrun tf_transform tf_transform
```
### 运行底盘坐标系的/cmd_vel速度指令控制底盘运动
```shell
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
```
### 通过发布/cmd_vel速度指令控制底盘运动
