#include "ros/ros_manager.h"

#include <rmw_microros/rmw_microros.h>

namespace {
constexpr uint32_t kAgentPingPeriodMs = 1000;   // health check while connected
constexpr uint32_t kWaitPingPeriodMs = 500;     // retry period while waiting
constexpr uint32_t kTimeSyncPeriodMs = 60000;   // re-sync clock with the agent
constexpr uint32_t kSpinTimeoutMs = 5;
}  // namespace

bool RosManager::addModule(RosModule* module) {
  if (!module || moduleCount_ >= kMaxModules) return false;
  modules_[moduleCount_++] = module;
  return true;
}

bool RosManager::begin(const Config& cfg) {
  cfg_ = cfg;
  if (cfg_.statusLedPin >= 0) pinMode(cfg_.statusLedPin, OUTPUT);
  setLed(false);
  return xTaskCreatePinnedToCore(&RosManager::taskEntry, "micro_ros", cfg_.taskStackBytes, this,
                                 cfg_.taskPriority, nullptr, cfg_.taskCore) == pdPASS;
}

void RosManager::taskEntry(void* arg) { static_cast<RosManager*>(arg)->run(); }

void RosManager::setLed(bool on) {
  if (cfg_.statusLedPin >= 0) digitalWrite(cfg_.statusLedPin, on ? HIGH : LOW);
}

void RosManager::run() {
  uint32_t lastPing = 0;
  uint32_t lastSync = 0;
  bool led = false;

  for (;;) {
    const uint32_t now = millis();

    switch (state_) {
      case State::WaitingAgent:
        if (now - lastPing >= kWaitPingPeriodMs) {
          lastPing = now;
          led = !led;
          setLed(led);
          if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) state_ = State::AgentAvailable;
        }
        break;

      case State::AgentAvailable:
        if (createEntities()) {
          rmw_uros_sync_session(1000);
          lastSync = lastPing = millis();
          setLed(true);
          state_ = State::Connected;
        } else {
          destroyEntities();
          state_ = State::WaitingAgent;
        }
        break;

      case State::Connected:
        if (now - lastPing >= kAgentPingPeriodMs) {
          lastPing = now;
          if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
            state_ = State::Disconnected;
            break;
          }
        }
        if (now - lastSync >= kTimeSyncPeriodMs) {
          lastSync = now;
          rmw_uros_sync_session(100);
        }
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(kSpinTimeoutMs));
        for (size_t i = 0; i < modulesCreated_; ++i) modules_[i]->update(millis());
        break;

      case State::Disconnected:
        destroyEntities();
        setLed(false);
        state_ = State::WaitingAgent;
        break;
    }

    // The serial transport busy-waits; give lower-priority tasks a chance to run.
    vTaskDelay(1);
  }
}

bool RosManager::createEntities() {
  allocator_ = rcl_get_default_allocator();

  initOptions_ = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&initOptions_, allocator_) != RCL_RET_OK) return false;
  initOptionsOk_ = true;
  if (rcl_init_options_set_domain_id(&initOptions_, cfg_.domainId) != RCL_RET_OK) return false;

  if (rclc_support_init_with_options(&support_, 0, nullptr, &initOptions_, &allocator_) != RCL_RET_OK)
    return false;
  supportOk_ = true;

  node_ = rcl_get_zero_initialized_node();
  if (rclc_node_init_default(&node_, cfg_.nodeName, cfg_.nodeNamespace, &support_) != RCL_RET_OK)
    return false;
  nodeOk_ = true;

  size_t handles = 0;
  for (size_t i = 0; i < moduleCount_; ++i) handles += modules_[i]->executorHandles();
  if (handles == 0) handles = 1;  // rclc requires at least one

  executor_ = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor_, &support_.context, handles, &allocator_) != RCL_RET_OK)
    return false;
  executorOk_ = true;

  modulesCreated_ = 0;
  for (size_t i = 0; i < moduleCount_; ++i) {
    // count it first so destroy() also cleans up a partially created module
    ++modulesCreated_;
    if (!modules_[i]->create(&node_, &executor_)) return false;
  }
  return true;
}

void RosManager::destroyEntities() {
  if (supportOk_) {
    // agent is probably gone: don't wait for it to confirm each deletion
    rmw_context_t* rmwContext = rcl_context_get_rmw_context(&support_.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmwContext, 0);
  }

  for (size_t i = modulesCreated_; i > 0; --i) modules_[i - 1]->destroy(&node_);
  modulesCreated_ = 0;

  if (executorOk_) (void)rclc_executor_fini(&executor_);
  if (nodeOk_) (void)rcl_node_fini(&node_);
  if (supportOk_) (void)rclc_support_fini(&support_);
  if (initOptionsOk_) (void)rcl_init_options_fini(&initOptions_);

  executorOk_ = nodeOk_ = supportOk_ = initOptionsOk_ = false;
}
