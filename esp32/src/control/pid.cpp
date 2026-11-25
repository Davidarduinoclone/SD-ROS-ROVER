#include "control/pid.h"

namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

void Pid::configure(const PidGains& gains, float outMin, float outMax) {
  gains_ = gains;
  outMin_ = outMin;
  outMax_ = outMax;
  reset();
}

void Pid::reset() {
  integral_ = 0.0f;
  prevMeasured_ = 0.0f;
  first_ = true;
}

float Pid::update(float setpoint, float measured, float dt) {
  if (dt <= 0.0f) return 0.0f;

  const float error = setpoint - measured;

  // derivative on measurement avoids a kick when the setpoint jumps
  const float derivative = first_ ? 0.0f : (measured - prevMeasured_) / dt;
  prevMeasured_ = measured;
  first_ = false;

  const float ff = gains_.kff * setpoint;
  const float p = gains_.kp * error;
  const float d = -gains_.kd * derivative;

  float newIntegral = integral_ + gains_.ki * error * dt;
  float out = ff + p + newIntegral + d;

  // anti-windup: if saturated, only let the integral move back toward the range
  if (out > outMax_) {
    out = outMax_;
    if (error > 0.0f) newIntegral = integral_;
  } else if (out < outMin_) {
    out = outMin_;
    if (error < 0.0f) newIntegral = integral_;
  }
  integral_ = clampf(newIntegral, outMin_, outMax_);

  return out;
}
