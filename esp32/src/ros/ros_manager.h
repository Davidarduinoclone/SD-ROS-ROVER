#pragma once
// Runs the micro-ROS connection in its own FreeRTOS task:
//   wait for agent -> create node + modules -> spin -> agent lost -> destroy -> wait ...
// so the ESP32 can be powered before the agent starts and survives agent restarts.
//
// The transport (serial or Wi-Fi) must be set in setup() before begin().

#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "ros/ros_module.h"

class RosManager {
 public:
  static constexpr size_t kMaxModules = 8;

  struct Config {
    const char* nodeName;
    const char* nodeNamespace;  // "" for none
    size_t domainId;            // must match ROS_DOMAIN_ID on the agent side
    int statusLedPin;           // -1 to disable. Blinks = waiting for agent, solid = connected
    uint32_t taskStackBytes;
    UBaseType_t taskPriority;
    BaseType_t taskCore;
  };

  // Register before begin(). Returns false if the module table is full.
  bool addModule(RosModule* module);

  bool begin(const Config& cfg);

  bool connected() const { return state_ == State::Connected; }

 private:
  enum class State { WaitingAgent, AgentAvailable, Connected, Disconnected };

  static void taskEntry(void* arg);
  void run();
  bool createEntities();
  void destroyEntities();
  void setLed(bool on);

  Config cfg_{};
  RosModule* modules_[kMaxModules] = {};
  size_t moduleCount_ = 0;

  volatile State state_ = State::WaitingAgent;

  rcl_allocator_t allocator_{};
  rcl_init_options_t initOptions_{};
  rclc_support_t support_{};
  rcl_node_t node_{};
  rclc_executor_t executor_{};

  bool initOptionsOk_ = false;
  bool supportOk_ = false;
  bool nodeOk_ = false;
  bool executorOk_ = false;
  size_t modulesCreated_ = 0;
};
