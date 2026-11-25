#pragma once
// Velocity PID with feed-forward, derivative-on-measurement and anti-windup.
//
//   output = kff * setpoint + kp * error + ki * integral(error) - kd * d(measured)/dt
//
// For a wheel: setpoint/measured are rad/s, output is PWM duty in [-1, 1].
// kff ~= 1 / (wheel speed at full duty) does most of the work; the PID
// only corrects the remaining error.

struct PidGains {
  float kp;
  float ki;
  float kd;
  float kff;
};

class Pid {
 public:
  void configure(const PidGains& gains, float outMin, float outMax);
  void setGains(const PidGains& gains) { gains_ = gains; }
  float update(float setpoint, float measured, float dt);
  void reset();

 private:
  PidGains gains_{};
  float outMin_ = -1.0f;
  float outMax_ = 1.0f;
  float integral_ = 0.0f;
  float prevMeasured_ = 0.0f;
  bool first_ = true;
};
