#pragma once
// micro-ROS interface for the Drivetrain.
//
//   subscribes  cmd_vel                  geometry_msgs/Twist          -> Drivetrain::setCommand
//   publishes   odom                     nav_msgs/Odometry            wheel odometry
//   publishes   /tf (optional)           tf2_msgs/TFMessage           odom -> base_link
//   publishes   drivetrain/wheel_status  std_msgs/Float32MultiArray   [target, measured, duty] per wheel
//
// Topic names without a leading '/' are placed under the node namespace.

#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <tf2_msgs/msg/tf_message.h>

#include "drivetrain/drivetrain.h"
#include "ros/ros_module.h"

class DrivetrainRos : public RosModule {
 public:
  struct Config {
    const char* cmdVelTopic;
    const char* odomTopic;
    const char* wheelStatusTopic;
    const char* odomFrame;
    const char* baseFrame;
    bool publishTf;          // turn off if robot_localization (EKF) publishes odom -> base_link
    uint32_t odomPeriodMs;
    uint32_t wheelStatusPeriodMs;  // 0 disables the wheel_status topic
  };

  DrivetrainRos(Drivetrain& drivetrain, const Config& cfg);

  const char* name() const override { return "drivetrain"; }
  size_t executorHandles() const override { return 1; }  // cmd_vel subscription
  bool create(rcl_node_t* node, rclc_executor_t* executor) override;
  void destroy(rcl_node_t* node) override;
  void update(uint32_t nowMs) override;

 private:
  static void onCmdVel(const void* msg, void* context);
  void publishOdom();
  void publishWheelStatus();

  Drivetrain& drivetrain_;
  Config cfg_;

  rcl_subscription_t cmdVelSub_{};
  rcl_publisher_t odomPub_{};
  rcl_publisher_t tfPub_{};
  rcl_publisher_t wheelPub_{};
  bool cmdVelOk_ = false, odomOk_ = false, tfOk_ = false, wheelOk_ = false;

  geometry_msgs__msg__Twist cmdVelMsg_{};
  nav_msgs__msg__Odometry odomMsg_{};
  geometry_msgs__msg__TransformStamped tfTransform_{};
  tf2_msgs__msg__TFMessage tfMsg_{};
  std_msgs__msg__Float32MultiArray wheelMsg_{};
  float wheelData_[Drivetrain::kMaxWheels * 3] = {};

  uint32_t lastOdomMs_ = 0;
  uint32_t lastWheelMs_ = 0;
};
