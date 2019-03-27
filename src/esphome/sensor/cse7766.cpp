#include "esphome/defines.h"

#ifdef USE_CSE7766

#include "esphome/sensor/cse7766.h"
#include "esphome/log.h"

ESPHOME_NAMESPACE_BEGIN

namespace sensor {

static const char *TAG = "sensor.cse7766";

void CSE7766Component::loop() {
  const uint32_t now = millis();
  if (now - this->last_transmission_ >= 500) {
    // last transmission too long ago. Reset RX index.
    this->raw_data_index_ = 0;
  }

  if (this->available() == 0)
    return;

  this->last_transmission_ = now;
  while (this->available() != 0) {
    this->read_byte(&this->raw_data_[this->raw_data_index_]);
    if (!this->check_byte_()) {
      if (this->raw_data_index_ == 1 && this->raw_data_[0] == 0x5A) {
        // if we missed a byte somewhere we will constantly bail out
        // on header2. But because the message size is even we will never
        // re-sync. solution: manually consume a byte so that the next full packet
        // can be read.
        uint8_t temp;
        this->read_byte(&temp);
      }

      this->raw_data_index_ = 0;
      this->status_set_warning();
      continue;
    }

    if (this->raw_data_index_ == 23) {
      this->parse_data_();
      this->status_clear_warning();
    }

    this->raw_data_index_ = (this->raw_data_index_ + 1) % 24;
  }
}
float CSE7766Component::get_setup_priority() const { return setup_priority::HARDWARE_LATE; }
bool CSE7766Component::check_byte_() {
  uint8_t index = this->raw_data_index_;
  uint8_t byte = this->raw_data_[index];
  if (index == 0) {
    // Header, must be 0x55 (all good) or 0xFX
    if (byte == 0x55)
      return true;
    if ((byte & 0xF0) == 0xF0)
      return true;
    if (byte == 0xAA)
      return true;
    ESP_LOGV(TAG, "Invalid Header 1: 0x%02X", byte);
    return false;
  }

  if (index == 1) {
    if (byte != 0x5A) {
      ESP_LOGV(TAG, "Invalid Header 2 Start: 0x%02X!", byte);
      return false;
    }

    return true;
  }

  if (index < 23)
    return true;

  uint8_t checksum = 0;
  for (uint8_t i = 2; i < 23; i++)
    checksum += this->raw_data_[i];

  if (checksum != this->raw_data_[23]) {
    ESP_LOGW(TAG, "Invalid checksum from CSE7766: 0x%02X != 0x%02X", checksum, this->raw_data_[23]);
    return false;
  }

  return true;
}
void CSE7766Component::parse_data_() {
  ESP_LOGVV(TAG, "CSE7766 Data: ");
  for (uint8_t i = 0; i < 23; i++) {
    ESP_LOGVV(TAG, "  i=%u: 0b" BYTE_TO_BINARY_PATTERN " (0x%02X)", i, BYTE_TO_BINARY(this->raw_data_[i]),
              this->raw_data_[i]);
  }

  uint8_t header1 = this->raw_data_[0];
  uint32_t voltage_calib = this->get_24_bit_uint_(2);
  uint32_t voltage_cycle = this->get_24_bit_uint_(5);
  uint32_t current_calib = this->get_24_bit_uint_(8);
  uint32_t current_cycle = this->get_24_bit_uint_(11);
  uint32_t power_calib = this->get_24_bit_uint_(14);
  uint32_t power_cycle = this->get_24_bit_uint_(17);

  uint8_t adj = this->raw_data_[20];

  bool header_power_ok = true;
  bool header_voltage_ok = true;
  bool header_current_ok = true;

  if (header1 == 0x55) {
    // All good
  } else if (header1 == 0xAA) {
    ESP_LOGW(TAG, "CSE7766 reports abnormal hardware: (0x%02X)", header1);
    ESP_LOGW(TAG, "  Coefficient storage area is abnormal.");
    return;
  } else if (header1 & 0xF0) {
    if (header1 & 0x08) {
      header_voltage_ok = false;
    }
    if (header1 & 0x04) {
      header_current_ok = false;
    }
    if (header1 & 0x02) {
      header_power_ok = false;
    }
  }

  bool voltage_ok = (adj & 0x40) && header_voltage_ok;
  bool current_ok = (adj & 0x20) && header_current_ok;
  bool power_ok = (adj & 0x10) && header_power_ok;

  if (!power_ok || power_calib == 0)
    // power sensor is more accurate than current. If power sensor reads 0
    // make sure current sensor also reads 0
    current_ok = false;

  if (voltage_ok) {
    float voltage = voltage_calib / float(voltage_cycle);
    this->read_voltage_ = voltage;
  }

  if (current_ok) {
    float current = current_calib / float(current_cycle);
    this->read_current_ = current;
  }

  if (power_ok) {
    float active_power = power_calib / float(power_cycle);
    this->read_power_ = active_power;
  }

  if (this->read_power_ > 1000.0f) {
    ESP_LOGW(TAG, "Abnormal CSE776 data:");
    for (uint8_t i = 0; i < 23; i++) {
      ESP_LOGW(TAG, "  i=%u: 0b" BYTE_TO_BINARY_PATTERN " (0x%02X)", i, BYTE_TO_BINARY(this->raw_data_[i]),
                this->raw_data_[i]);
    }
  }
}
void CSE7766Component::update() {
  ESP_LOGD(TAG, "Got voltage=%.1fV current=%.1fA power=%.1fW", this->read_voltage_, this->read_current_,
           this->read_power_);

  if (this->voltage_ != nullptr)
    this->voltage_->publish_state(this->read_voltage_);
  if (this->current_ != nullptr)
    this->current_->publish_state(this->read_current_);
  if (this->power_ != nullptr)
    this->power_->publish_state(this->read_power_);

  // reset values
  this->read_voltage_ = 0.0f;
  this->read_current_ = 0.0f;
  this->read_power_ = 0.0f;
}
uint32_t CSE7766Component::get_24_bit_uint_(uint8_t start_index) {
  return (uint32_t(this->raw_data_[start_index]) << 16) | (uint32_t(this->raw_data_[start_index + 1]) << 8) |
         uint32_t(this->raw_data_[start_index + 2]);
}

CSE7766Component::CSE7766Component(UARTComponent *parent, uint32_t update_interval)
    : UARTDevice(parent), PollingComponent(update_interval) {}
CSE7766VoltageSensor *CSE7766Component::make_voltage_sensor(const std::string &name) {
  return this->voltage_ = new CSE7766VoltageSensor(name);
}
CSE7766CurrentSensor *CSE7766Component::make_current_sensor(const std::string &name) {
  return this->current_ = new CSE7766CurrentSensor(name);
}
CSE7766PowerSensor *CSE7766Component::make_power_sensor(const std::string &name) {
  return this->power_ = new CSE7766PowerSensor(name);
}
void CSE7766Component::dump_config() {
  ESP_LOGCONFIG(TAG, "CSE7766:");
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Voltage", this->voltage_);
  LOG_SENSOR("  ", "Current", this->current_);
  LOG_SENSOR("  ", "Power", this->power_);
}

}  // namespace sensor

ESPHOME_NAMESPACE_END

#endif  // USE_CSE7766
