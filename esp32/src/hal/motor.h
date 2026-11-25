#pragma once
// H-bridge DC motor driven by one LEDC PWM channel.
//
// Supports the two common driver wirings:
//   * IN1 / IN2 / PWM  (TB6612FNG, L298N with ENA jumper removed, DRV8833 via PWM on EN)
//       in1 = direction A, in2 = direction B, pwm = speed
//   * DIR / PWM        (Cytron MD10C/MD13S, many BTS7960 adapters)
//       in1 = DIR, in2 = -1, pwm = speed

#include <Arduino.h>

struct MotorPins {
  int pwm;          // PWM / enable pin
  int in1;          // IN1, or DIR for DIR/PWM drivers
  int in2;          // IN2, or -1 for DIR/PWM drivers
  int ledcChannel;  // 0..15, must be unique per motor
  bool inverted;    // flip direction so positive duty drives the rover forward
};

class HBridgeMotor {
 public:
  void begin(const MotorPins& pins, uint32_t pwmFrequency, uint8_t pwmResolutionBits);

  // duty in [-1, 1]; sign = direction. 0 lets the motor coast.
  void setDuty(float duty);

 private:
  MotorPins pins_{};
  uint32_t maxLevel_ = 0;
  bool ready_ = false;
};
