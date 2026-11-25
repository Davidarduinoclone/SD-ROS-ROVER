#pragma once
// Differential / skid-steer drivetrain.
//
// Owns N wheels (motor + encoder + PID), each assigned to the left or right
// side. 2 wheels = differential drive, 4 or 6 = skid steer: only the wheel
// table in config.h changes.
//
// Runs its own FreeRTOS task at a fixed rate:
//   cmd (v, w) -> wheel speed targets -> accel limit -> PID -> PWM
//   encoders -> wheel speeds -> odometry
// All public methods are thread-safe (the micro-ROS task calls them).

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "control/pid.h"
#include "hal/encoder.h"
#include "hal/motor.h"

enum class Side : uint8_t { Left, Right };

struct WheelConfig {
  const char* name;
  Side side;
  MotorPins motor;
  EncoderPins encoder;
};

struct DrivetrainConfig {
  float wheelRadius;          // m
  float trackWidth;           // m, between left and right wheel centres (tune for skid steer)
  float countsPerRev;         // encoder counts per WHEEL revolution (4 x PPR x gear ratio)
  float maxWheelSpeed;        // rad/s, commands are scaled down to stay under this
  float maxWheelAccel;        // rad/s^2, ramp limit on wheel targets
  uint32_t cmdTimeoutMs;      // stop if no command for this long
  uint32_t controlPeriodMs;   // control loop period
  float velocityFilterAlpha;  // 0..1 low-pass on measured speed (1 = no filter)
  PidGains pid;
  uint32_t pwmFrequency;      // Hz
  uint8_t pwmResolutionBits;
  uint16_t encoderGlitchFilter;
  UBaseType_t taskPriority;
  BaseType_t taskCore;
};

struct Odometry {
  float x;        // m, in the odom frame
  float y;        // m
  float theta;    // rad, [-pi, pi]
  float linear;   // m/s, body frame
  float angular;  // rad/s
};

struct WheelStatus {
  float target;    // rad/s
  float measured;  // rad/s
  float duty;      // -1..1
};

class Drivetrain {
 public:
  static constexpr size_t kMaxWheels = 6;

  // Sets up the hardware and starts the control task. Returns false on bad config.
  bool begin(const DrivetrainConfig& cfg, const WheelConfig* wheels, size_t count);

  // Body velocity command: linear m/s (forward +), angular rad/s (CCW +).
  void setCommand(float linear, float angular);

  Odometry odometry() const;
  void resetOdometry();

  size_t wheelCount() const { return count_; }
  WheelStatus wheelStatus(size_t index) const;

  void setPidGains(const PidGains& gains);

 private:
  struct Wheel {
    WheelConfig cfg{};
    HBridgeMotor motor;
    QuadratureEncoder encoder;
    Pid pid;
    int64_t lastCount = 0;
    float measured = 0.0f;
    float target = 0.0f;
    float duty = 0.0f;
  };

  static void taskEntry(void* arg);
  void run();
  void step(float dt);

  DrivetrainConfig cfg_{};
  Wheel wheels_[kMaxWheels];
  size_t count_ = 0;

  mutable SemaphoreHandle_t mutex_ = nullptr;

  // shared with other tasks, protected by mutex_
  float cmdLinear_ = 0.0f;
  float cmdAngular_ = 0.0f;
  uint32_t lastCmdMs_ = 0;
  bool haveCmd_ = false;
  bool pendingGains_ = false;
  PidGains newGains_{};
  bool pendingOdomReset_ = false;
  Odometry odom_{};
  WheelStatus status_[kMaxWheels]{};
};
