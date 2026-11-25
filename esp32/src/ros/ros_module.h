#pragma once
// Interface every micro-ROS feature (drivetrain, IMU, shaker, battery, ...) implements.
//
// RosManager owns the node and executor and calls these hooks from its own
// task, which is the only task that touches micro-ROS (micro-ROS is not
// thread-safe). Hardware work belongs in the module's own FreeRTOS task and
// is exchanged with the hooks below through thread-safe getters/setters.
//
// Lifecycle (repeats every time the agent connects / disconnects):
//   create()  -> update() ... update() -> destroy()

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

class RosModule {
 public:
  virtual ~RosModule() = default;

  virtual const char* name() const = 0;

  // Executor handles this module adds (one per subscription, service, client
  // or timer). Publishers need none.
  virtual size_t executorHandles() const = 0;

  // Create publishers/subscriptions and add callbacks to the executor.
  virtual bool create(rcl_node_t* node, rclc_executor_t* executor) = 0;

  // Destroy everything create() made. Must be safe after a partial create().
  virtual void destroy(rcl_node_t* node) = 0;

  // Called every loop of the ROS task while connected. Publish from here.
  virtual void update(uint32_t nowMs) { (void)nowMs; }
};
