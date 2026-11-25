#include "drivetrain/drivetrain.h"

#include <esp_timer.h>
#include <math.h>

namespace {
constexpr float kTwoPi = 6.28318530718f;
// below this measured speed (rad/s) with a zero target, the wheel is "stopped"
constexpr float kStoppedSpeed = 0.05f;

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

float wrapAngle(float a) {
  while (a > PI) a -= kTwoPi;
  while (a < -PI) a += kTwoPi;
  return a;
}

class Lock {
 public:
  explicit Lock(SemaphoreHandle_t m) : m_(m) { xSemaphoreTake(m_, portMAX_DELAY); }
  ~Lock() { xSemaphoreGive(m_); }
 private:
  SemaphoreHandle_t m_;
};
}  // namespace

bool Drivetrain::begin(const DrivetrainConfig& cfg, const WheelConfig* wheels, size_t count) {
  if (count == 0 || count > kMaxWheels) return false;
  if (cfg.wheelRadius <= 0 || cfg.trackWidth <= 0 || cfg.countsPerRev <= 0) return false;

  cfg_ = cfg;
  count_ = count;
  mutex_ = xSemaphoreCreateMutex();
  if (!mutex_) return false;

  for (size_t i = 0; i < count_; ++i) {
    Wheel& w = wheels_[i];
    w.cfg = wheels[i];
    w.motor.begin(w.cfg.motor, cfg_.pwmFrequency, cfg_.pwmResolutionBits);
    if (!w.encoder.begin(w.cfg.encoder, cfg_.encoderGlitchFilter)) return false;
    w.pid.configure(cfg_.pid, -1.0f, 1.0f);
    w.lastCount = w.encoder.count();
  }

  return xTaskCreatePinnedToCore(&Drivetrain::taskEntry, "drivetrain", 4096, this,
                                 cfg_.taskPriority, nullptr, cfg_.taskCore) == pdPASS;
}

void Drivetrain::setCommand(float linear, float angular) {
  Lock lock(mutex_);
  cmdLinear_ = linear;
  cmdAngular_ = angular;
  lastCmdMs_ = millis();
  haveCmd_ = true;
}

Odometry Drivetrain::odometry() const {
  Lock lock(mutex_);
  return odom_;
}

void Drivetrain::resetOdometry() {
  Lock lock(mutex_);
  pendingOdomReset_ = true;
}

WheelStatus Drivetrain::wheelStatus(size_t index) const {
  if (index >= count_) return {};
  Lock lock(mutex_);
  return status_[index];
}

void Drivetrain::setPidGains(const PidGains& gains) {
  Lock lock(mutex_);
  newGains_ = gains;
  pendingGains_ = true;
}

void Drivetrain::taskEntry(void* arg) { static_cast<Drivetrain*>(arg)->run(); }

void Drivetrain::run() {
  const TickType_t period = pdMS_TO_TICKS(cfg_.controlPeriodMs);
  TickType_t lastWake = xTaskGetTickCount();
  int64_t prevUs = esp_timer_get_time();

  for (;;) {
    vTaskDelayUntil(&lastWake, period);
    const int64_t nowUs = esp_timer_get_time();
    const float dt = (nowUs - prevUs) * 1e-6f;
    prevUs = nowUs;
    if (dt > 0.0f) step(dt);
  }
}

void Drivetrain::step(float dt) {
  // ---- read shared inputs ----
  float linear, angular;
  bool resetOdom;
  {
    Lock lock(mutex_);
    const bool fresh = haveCmd_ && (millis() - lastCmdMs_) <= cfg_.cmdTimeoutMs;
    linear = fresh ? cmdLinear_ : 0.0f;
    angular = fresh ? cmdAngular_ : 0.0f;
    if (pendingGains_) {
      for (size_t i = 0; i < count_; ++i) wheels_[i].pid.setGains(newGains_);
      pendingGains_ = false;
    }
    resetOdom = pendingOdomReset_;
    pendingOdomReset_ = false;
  }

  // ---- inverse kinematics: body velocity -> wheel angular speed (rad/s) ----
  const float halfTrack = cfg_.trackWidth * 0.5f;
  float leftTarget = (linear - angular * halfTrack) / cfg_.wheelRadius;
  float rightTarget = (linear + angular * halfTrack) / cfg_.wheelRadius;

  // scale both sides together so the turn radius is kept when saturating
  const float biggest = fmaxf(fabsf(leftTarget), fabsf(rightTarget));
  if (biggest > cfg_.maxWheelSpeed) {
    const float s = cfg_.maxWheelSpeed / biggest;
    leftTarget *= s;
    rightTarget *= s;
  }

  // ---- per-wheel control ----
  const float maxStep = cfg_.maxWheelAccel * dt;
  const float radPerCount = kTwoPi / cfg_.countsPerRev;
  float leftDist = 0.0f, rightDist = 0.0f;
  int leftN = 0, rightN = 0;

  for (size_t i = 0; i < count_; ++i) {
    Wheel& w = wheels_[i];

    const int64_t c = w.encoder.count();
    const float dAngle = static_cast<float>(c - w.lastCount) * radPerCount;
    w.lastCount = c;

    const float rawSpeed = dAngle / dt;
    const float a = cfg_.velocityFilterAlpha;
    w.measured = a * rawSpeed + (1.0f - a) * w.measured;

    const float desired = (w.cfg.side == Side::Left) ? leftTarget : rightTarget;
    w.target += clampf(desired - w.target, -maxStep, maxStep);

    if (w.target == 0.0f && fabsf(w.measured) < kStoppedSpeed) {
      w.pid.reset();  // avoid integral creep / motor hum while parked
      w.duty = 0.0f;
    } else {
      w.duty = w.pid.update(w.target, w.measured, dt);
    }
    w.motor.setDuty(w.duty);

    const float dist = dAngle * cfg_.wheelRadius;
    if (w.cfg.side == Side::Left) {
      leftDist += dist;
      ++leftN;
    } else {
      rightDist += dist;
      ++rightN;
    }
  }

  if (leftN) leftDist /= leftN;
  if (rightN) rightDist /= rightN;

  // ---- forward kinematics: odometry (midpoint integration) ----
  const float ds = 0.5f * (leftDist + rightDist);
  const float dTheta = (rightDist - leftDist) / cfg_.trackWidth;

  Lock lock(mutex_);
  if (resetOdom) odom_ = {};
  const float heading = odom_.theta + 0.5f * dTheta;
  odom_.x += ds * cosf(heading);
  odom_.y += ds * sinf(heading);
  odom_.theta = wrapAngle(odom_.theta + dTheta);
  odom_.linear = ds / dt;
  odom_.angular = dTheta / dt;

  for (size_t i = 0; i < count_; ++i) {
    status_[i] = {wheels_[i].target, wheels_[i].measured, wheels_[i].duty};
  }
}
