#pragma once
// Quadrature encoder read by the ESP32 PCNT (pulse counter) hardware.
//
// Counts every edge of both channels (4x decoding), so
//   counts per motor revolution = 4 * encoder PPR.
// The hardware counter is 16-bit; overflows are accumulated in an interrupt
// into a 64-bit total, so count() never wraps in practice.
// The ESP32 has 8 PCNT units, so up to 8 encoders.

#include <Arduino.h>
#include <driver/pcnt.h>

struct EncoderPins {
  int a;
  int b;
  bool inverted;  // flip sign so forward motion counts up
};

class QuadratureEncoder {
 public:
  // glitchFilter: ignore pulses shorter than this many APB cycles (12.5 ns each, max 1023)
  bool begin(const EncoderPins& pins, uint16_t glitchFilter = 100);

  // Total counts since begin(). Safe to call from any task.
  int64_t count();

 private:
  static void IRAM_ATTR onLimit(void* arg);

  EncoderPins pins_{};
  pcnt_unit_t unit_ = PCNT_UNIT_MAX;
  volatile int64_t overflow_ = 0;
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;

  static int nextUnit_;
  static bool isrServiceInstalled_;
};
