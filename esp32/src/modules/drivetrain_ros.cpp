#include "modules/drivetrain_ros.h"

#include <math.h>
#include <rmw_microros/rmw_microros.h>
#include <string.h>

namespace {
// Point a ROS string at constant text (no allocation; the text must outlive the message).
void setString(rosidl_runtime_c__String& s, const char* text) {
  s.data = const_cast<char*>(text);
  s.size = strlen(text);
  s.capacity = s.size + 1;
}

void stampNow(builtin_interfaces__msg__Time& stamp) {
  const int64_t ns = rmw_uros_epoch_nanos();  // agent-synchronised clock
  stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
}
}  // namespace

DrivetrainRos::DrivetrainRos(Drivetrain& drivetrain, const Config& cfg)
    : drivetrain_(drivetrain), cfg_(cfg) {
  // ---- odometry message: constant fields ----
  setString(odomMsg_.header.frame_id, cfg_.odomFrame);
  setString(odomMsg_.child_frame_id, cfg_.baseFrame);
  // Diagonal covariance (x, y, z, roll, pitch, yaw). z/roll/pitch unused on a ground robot.
  const double poseCov[6] = {0.01, 0.01, 1e6, 1e6, 1e6, 0.03};
  const double twistCov[6] = {0.01, 1e6, 1e6, 1e6, 1e6, 0.03};
  for (int i = 0; i < 6; ++i) {
    odomMsg_.pose.covariance[i * 7] = poseCov[i];
    odomMsg_.twist.covariance[i * 7] = twistCov[i];
  }

  // ---- TF message: one transform ----
  setString(tfTransform_.header.frame_id, cfg_.odomFrame);
  setString(tfTransform_.child_frame_id, cfg_.baseFrame);
  tfMsg_.transforms.data = &tfTransform_;
  tfMsg_.transforms.size = 1;
  tfMsg_.transforms.capacity = 1;

  // ---- wheel status: flat float array ----
  wheelMsg_.data.data = wheelData_;
  wheelMsg_.data.capacity = Drivetrain::kMaxWheels * 3;
  wheelMsg_.data.size = 0;
}

bool DrivetrainRos::create(rcl_node_t* node, rclc_executor_t* executor) {
  cmdVelSub_ = rcl_get_zero_initialized_subscription();
  if (rclc_subscription_init_default(&cmdVelSub_, node,
                                     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                     cfg_.cmdVelTopic) != RCL_RET_OK)
    return false;
  cmdVelOk_ = true;

  if (rclc_executor_add_subscription_with_context(executor, &cmdVelSub_, &cmdVelMsg_,
                                                  &DrivetrainRos::onCmdVel, this,
                                                  ON_NEW_DATA) != RCL_RET_OK)
    return false;

  odomPub_ = rcl_get_zero_initialized_publisher();
  if (rclc_publisher_init_default(&odomPub_, node,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                                  cfg_.odomTopic) != RCL_RET_OK)
    return false;
  odomOk_ = true;

  if (cfg_.publishTf) {
    tfPub_ = rcl_get_zero_initialized_publisher();
    if (rclc_publisher_init_default(&tfPub_, node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
                                    "/tf") != RCL_RET_OK)
      return false;
    tfOk_ = true;
  }

  if (cfg_.wheelStatusPeriodMs > 0) {
    wheelPub_ = rcl_get_zero_initialized_publisher();
    if (rclc_publisher_init_default(&wheelPub_, node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                                    cfg_.wheelStatusTopic) != RCL_RET_OK)
      return false;
    wheelOk_ = true;
  }

  return true;
}

void DrivetrainRos::destroy(rcl_node_t* node) {
  if (cmdVelOk_) (void)rcl_subscription_fini(&cmdVelSub_, node);
  if (odomOk_) (void)rcl_publisher_fini(&odomPub_, node);
  if (tfOk_) (void)rcl_publisher_fini(&tfPub_, node);
  if (wheelOk_) (void)rcl_publisher_fini(&wheelPub_, node);
  cmdVelOk_ = odomOk_ = tfOk_ = wheelOk_ = false;
}

void DrivetrainRos::onCmdVel(const void* msg, void* context) {
  const auto* twist = static_cast<const geometry_msgs__msg__Twist*>(msg);
  auto* self = static_cast<DrivetrainRos*>(context);
  self->drivetrain_.setCommand(static_cast<float>(twist->linear.x),
                               static_cast<float>(twist->angular.z));
}

void DrivetrainRos::update(uint32_t nowMs) {
  if (nowMs - lastOdomMs_ >= cfg_.odomPeriodMs) {
    lastOdomMs_ = nowMs;
    publishOdom();
  }
  if (wheelOk_ && nowMs - lastWheelMs_ >= cfg_.wheelStatusPeriodMs) {
    lastWheelMs_ = nowMs;
    publishWheelStatus();
  }
}

void DrivetrainRos::publishOdom() {
  const Odometry o = drivetrain_.odometry();

  stampNow(odomMsg_.header.stamp);
  odomMsg_.pose.pose.position.x = o.x;
  odomMsg_.pose.pose.position.y = o.y;
  odomMsg_.pose.pose.position.z = 0.0;
  // yaw-only quaternion
  odomMsg_.pose.pose.orientation.x = 0.0;
  odomMsg_.pose.pose.orientation.y = 0.0;
  odomMsg_.pose.pose.orientation.z = sin(o.theta * 0.5);
  odomMsg_.pose.pose.orientation.w = cos(o.theta * 0.5);
  odomMsg_.twist.twist.linear.x = o.linear;
  odomMsg_.twist.twist.angular.z = o.angular;
  (void)rcl_publish(&odomPub_, &odomMsg_, nullptr);

  if (tfOk_) {
    tfTransform_.header.stamp = odomMsg_.header.stamp;
    tfTransform_.transform.translation.x = o.x;
    tfTransform_.transform.translation.y = o.y;
    tfTransform_.transform.translation.z = 0.0;
    tfTransform_.transform.rotation = odomMsg_.pose.pose.orientation;
    (void)rcl_publish(&tfPub_, &tfMsg_, nullptr);
  }
}

void DrivetrainRos::publishWheelStatus() {
  const size_t n = drivetrain_.wheelCount();
  for (size_t i = 0; i < n; ++i) {
    const WheelStatus s = drivetrain_.wheelStatus(i);
    wheelData_[i * 3 + 0] = s.target;
    wheelData_[i * 3 + 1] = s.measured;
    wheelData_[i * 3 + 2] = s.duty;
  }
  wheelMsg_.data.size = n * 3;
  (void)rcl_publish(&wheelPub_, &wheelMsg_, nullptr);
}
