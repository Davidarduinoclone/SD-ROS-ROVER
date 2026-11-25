#include "hal/motor.h"

#include <math.h>

void HBridgeMotor::begin(const MotorPins& pins, uint32_t pwmFrequency, uint8_t pwmResolutionBits) {
  pins_ = pins;
  maxLevel_ = (1u << pwmResolutionBits) - 1u;

  ledcSetup(pins_.ledcChannel, pwmFrequency, pwmResolutionBits);
  ledcAttachPin(pins_.pwm, pins_.ledcChannel);

  pinMode(pins_.in1, OUTPUT);
  if (pins_.in2 >= 0) pinMode(pins_.in2, OUTPUT);

  ready_ = true;
  setDuty(0.0f);
}

void HBridgeMotor::setDuty(float duty) {
  if (!ready_) return;

  duty = constrain(duty, -1.0f, 1.0f);
  if (pins_.inverted) duty = -duty;

  const uint32_t level = static_cast<uint32_t>(lroundf(fabsf(duty) * maxLevel_));
  const bool forward = duty >= 0.0f;

  if (pins_.in2 >= 0) {
    if (level == 0) {
      // both low = coast on TB6612 / L298N
      digitalWrite(pins_.in1, LOW);
      digitalWrite(pins_.in2, LOW);
    } else {
      digitalWrite(pins_.in1, forward ? HIGH : LOW);
      digitalWrite(pins_.in2, forward ? LOW : HIGH);
    }
  } else {
    digitalWrite(pins_.in1, forward ? HIGH : LOW);
  }

  ledcWrite(pins_.ledcChannel, level);
}
