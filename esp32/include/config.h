#pragma once
// ============================================================================
//  Rover ESP32 configuration: every pin, dimension, gain and rate lives here.
//  Values marked  TODO(measure)  are placeholders; set them for your rover.
// ============================================================================

#include "drivetrain/drivetrain.h"
#include "modules/drivetrain_ros.h"
#include "ros/ros_manager.h"

namespace config {

// ------------------------------------------------------------------ micro-ROS
constexpr uint32_t kSerialBaud = 115200;  // must match the agent:  -b 115200

inline constexpr RosManager::Config kRos = {
    "esp32_drivetrain",  // node name
    "",                  // namespace, e.g. "sensor_rover" / "shaker_rover" for two rovers
    0,                   // ROS domain ID, must match ROS_DOMAIN_ID on the Pi
    2,                   // status LED pin (on-board LED on most dev boards), -1 = none
    12288,               // task stack (bytes)
    3,                   // task priority
    1,                   // core
};

// ----------------------------------------------------------------- wheels
// Pin rules (ESP32-WROOM):
//   * avoid 6-11 (flash), 1/3 (UART0 = micro-ROS), and strapping pins 0, 2, 5, 12, 15 for motors
//   * 34-39 are input-only and have NO internal pull-ups; good for encoders, but
//     open-collector encoders then need external pull-ups (e.g. 4.7k to 3.3V)
//   * encoder signals must be 3.3 V. Level-shift 5 V encoders.
//
// Motor: {pwm, in1, in2, ledcChannel, inverted}   in2 = -1 for DIR/PWM drivers
// Encoder: {A, B, inverted}
// A motor mounted mirrored on the right side usually needs both inverted flags set.
//
// To go 4WD skid steer, add rows (e.g. "left_rear", Side::Left, ...) with unique
// pins, LEDC channels (0-15) and encoders (max 8 PCNT units). Max 6 wheels.
inline constexpr WheelConfig kWheels[] = {
    // name     side         motor                       encoder
    {"left",  Side::Left,  {25, 26, 27, 0, false}, {34, 35, false}},  // TODO(measure)
    {"right", Side::Right, {33, 32,  4, 1, true},  {36, 39, true}},   // TODO(measure)
};
constexpr size_t kWheelCount = sizeof(kWheels) / sizeof(kWheels[0]);

// ----------------------------------------------------------------- drivetrain
inline constexpr DrivetrainConfig kDrivetrain = {
    0.050f,   // wheelRadius (m)                              TODO(measure)
    0.300f,   // trackWidth (m), centre-to-centre             TODO(measure)
    1320.0f,  // countsPerRev at the WHEEL: 4 x PPR x gear    TODO(measure) e.g. 4 x 11 x 30
    15.0f,    // maxWheelSpeed (rad/s), no-load speed at full duty, slightly reduced
    30.0f,    // maxWheelAccel (rad/s^2)
    500,      // cmdTimeoutMs: stop if cmd_vel stops arriving
    20,       // controlPeriodMs (50 Hz)
    0.5f,     // velocityFilterAlpha
    // PID gains, output is PWM duty (-1..1) from wheel speed (rad/s)
    // kff should be ~ 1 / (wheel speed at duty 1.0). Tune kp, then ki; kd usually 0.
    {0.05f,   // kp
     0.50f,   // ki
     0.0f,    // kd
     1.0f / 16.0f},  // kff  TODO(measure): 1 / measured full-duty speed
    20000,    // pwmFrequency (Hz), above hearing
    10,       // pwmResolutionBits (0..1023)
    100,      // encoderGlitchFilter (APB cycles, 12.5 ns each)
    5,        // control task priority (higher than micro-ROS)
    0,        // control task core
};

// ----------------------------------------------------------------- drivetrain topics
inline constexpr DrivetrainRos::Config kDrivetrainRos = {
    "cmd_vel",                  // subscribed
    "odom",                     // nav_msgs/Odometry
    "drivetrain/wheel_status",  // [target, measured, duty] x wheels, for PID tuning
    "odom",                     // odom frame
    "base_link",                // robot frame
    true,                       // publish odom -> base_link TF (false if an EKF does it)
    50,                         // odom period ms (20 Hz)
    100,                        // wheel_status period ms (10 Hz), 0 = off
};

}  // namespace config
