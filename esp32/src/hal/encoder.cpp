#include "hal/encoder.h"

namespace {
// Counter resets to 0 when it reaches +/- this value and raises an interrupt.
constexpr int16_t kLimit = 30000;
}  // namespace

int QuadratureEncoder::nextUnit_ = 0;
bool QuadratureEncoder::isrServiceInstalled_ = false;

bool QuadratureEncoder::begin(const EncoderPins& pins, uint16_t glitchFilter) {
  if (nextUnit_ >= PCNT_UNIT_MAX) return false;  // all 8 units used
  pins_ = pins;
  unit_ = static_cast<pcnt_unit_t>(nextUnit_++);

  // Channel 0: count edges on A, direction from B
  pcnt_config_t cfg = {};
  cfg.unit = unit_;
  cfg.channel = PCNT_CHANNEL_0;
  cfg.pulse_gpio_num = pins_.a;
  cfg.ctrl_gpio_num = pins_.b;
  cfg.pos_mode = PCNT_COUNT_DEC;
  cfg.neg_mode = PCNT_COUNT_INC;
  cfg.lctrl_mode = PCNT_MODE_REVERSE;
  cfg.hctrl_mode = PCNT_MODE_KEEP;
  cfg.counter_h_lim = kLimit;
  cfg.counter_l_lim = -kLimit;
  if (pcnt_unit_config(&cfg) != ESP_OK) return false;

  // Channel 1: count edges on B, direction from A
  cfg.channel = PCNT_CHANNEL_1;
  cfg.pulse_gpio_num = pins_.b;
  cfg.ctrl_gpio_num = pins_.a;
  cfg.pos_mode = PCNT_COUNT_INC;
  cfg.neg_mode = PCNT_COUNT_DEC;
  if (pcnt_unit_config(&cfg) != ESP_OK) return false;

  pcnt_set_filter_value(unit_, glitchFilter > 1023 ? 1023 : glitchFilter);
  pcnt_filter_enable(unit_);

  pcnt_event_enable(unit_, PCNT_EVT_H_LIM);
  pcnt_event_enable(unit_, PCNT_EVT_L_LIM);

  pcnt_counter_pause(unit_);
  pcnt_counter_clear(unit_);

  if (!isrServiceInstalled_) {
    const esp_err_t err = pcnt_isr_service_install(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return false;  // INVALID_STATE = already installed
    isrServiceInstalled_ = true;
  }
  if (pcnt_isr_handler_add(unit_, &QuadratureEncoder::onLimit, this) != ESP_OK) return false;
  pcnt_intr_enable(unit_);

  pcnt_counter_resume(unit_);
  return true;
}

void IRAM_ATTR QuadratureEncoder::onLimit(void* arg) {
  auto* self = static_cast<QuadratureEncoder*>(arg);
  uint32_t status = 0;
  pcnt_get_event_status(self->unit_, &status);

  portENTER_CRITICAL_ISR(&self->mux_);
  if (status & PCNT_EVT_H_LIM) self->overflow_ += kLimit;
  if (status & PCNT_EVT_L_LIM) self->overflow_ -= kLimit;
  portEXIT_CRITICAL_ISR(&self->mux_);
}

int64_t QuadratureEncoder::count() {
  if (unit_ == PCNT_UNIT_MAX) return 0;

  int16_t raw = 0;
  portENTER_CRITICAL(&mux_);
  pcnt_get_counter_value(unit_, &raw);
  const int64_t total = overflow_ + raw;
  portEXIT_CRITICAL(&mux_);

  return pins_.inverted ? -total : total;
}
