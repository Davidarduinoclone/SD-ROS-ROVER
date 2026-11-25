// Rover drivetrain firmware: ESP32 + micro-ROS + FreeRTOS
//
// Tasks:
//   drivetrain  (core 0, prio 5)  50 Hz PID loop: encoders -> PID -> PWM, odometry
//   micro_ros   (core 1, prio 3)  agent connection, cmd_vel in, odom / tf / wheel_status out
//
// Adding a feature (IMU, more motors, shaker, ...): create a hardware class
// plus a RosModule, then construct and register it below. See README.md.

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "config.h"
#include "drivetrain/drivetrain.h"
#include "modules/drivetrain_ros.h"
#include "ros/ros_manager.h"

namespace {
Drivetrain drivetrain;
DrivetrainRos drivetrainRos(drivetrain, config::kDrivetrainRos);
RosManager rosManager;

// Hardware init failed (bad config, out of PCNT units, ...). Motors stay off; LED blinks fast.
[[noreturn]] void fail() {
  const int led = config::kRos.statusLedPin;
  if (led >= 0) pinMode(led, OUTPUT);
  for (;;) {
    if (led >= 0) digitalWrite(led, !digitalRead(led));
    delay(100);
  }
}
}  // namespace

void setup() {
  // ---- transport ----
  Serial.begin(config::kSerialBaud);
  set_microros_serial_transports(Serial);
  // Wi-Fi instead (set board_microros_transport = wifi in platformio.ini):
  //   IPAddress agentIp(192, 168, 1, 10);
  //   set_microros_wifi_transports("ssid", "password", agentIp, 8888);

  // ---- hardware ----
  if (!drivetrain.begin(config::kDrivetrain, config::kWheels, config::kWheelCount)) fail();

  // ---- ROS modules ----
  rosManager.addModule(&drivetrainRos);
  // rosManager.addModule(&imuRos);   // future modules go here

  if (!rosManager.begin(config::kRos)) fail();
}

void loop() {
  // Everything runs in FreeRTOS tasks.
  vTaskDelete(nullptr);
}
