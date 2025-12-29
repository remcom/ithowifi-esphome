#pragma once

#include "esphome/core/component.h"
#include "esphome/components/fan/fan.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/itho_i2c_bus/itho_i2c_bus.h"
#include <vector>

namespace esphome {
namespace itho_i2c {

enum IthoCommand {
  ITHO_LOW = 0,
  ITHO_MEDIUM = 1,
  ITHO_HIGH = 2,
  ITHO_FULL = 3,
  ITHO_TIMER1 = 4,
  ITHO_TIMER2 = 5,
  ITHO_TIMER3 = 6,
  ITHO_JOIN = 7,
  ITHO_LEAVE = 8,
  ITHO_AUTO = 9,
  ITHO_AUTONIGHT = 10,
  ITHO_COOK30 = 11,
  ITHO_COOK60 = 12,
};

class IthoI2CComponent : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Set the custom I2C bus
  void set_bus(IthoI2CBus *bus) { this->bus_ = bus; }

  // Fan control
  void set_speed(uint8_t speed);
  void send_remote_command(IthoCommand command, uint8_t remote_index = 0);
  void set_pwm_speed(uint16_t value);

  // Device queries
  void query_device_type();
  void query_status_format();
  void query_status();
  void query_counters();
  void query_measurements();

  // Sensors
  void set_temperature_sensor(sensor::Sensor *sensor) { this->temperature_sensor_ = sensor; }
  void set_humidity_sensor(sensor::Sensor *sensor) { this->humidity_sensor_ = sensor; }
  void set_fan_speed_sensor(sensor::Sensor *sensor) { this->fan_speed_sensor_ = sensor; }
  void set_fan_setpoint_sensor(sensor::Sensor *sensor) { this->fan_setpoint_sensor_ = sensor; }
  void set_fan_speed_rpm_sensor(sensor::Sensor *sensor) { this->fan_speed_rpm_sensor_ = sensor; }
  void set_device_type_sensor(text_sensor::TextSensor *sensor) { this->device_type_sensor_ = sensor; }

  // Configuration
  void set_remote_id(uint8_t id1, uint8_t id2, uint8_t id3) {
    remote_id_[0] = id1;
    remote_id_[1] = id2;
    remote_id_[2] = id3;
  }

 protected:
  // I2C helpers
  bool send_i2c_command(const uint8_t *cmd, size_t len, bool release_master = false);
  bool send_i2c_bytes(const uint8_t *buf, size_t len, bool release_master = false);
  bool read_i2c_response(uint8_t addr, uint8_t *data, size_t size);
  size_t read_i2c_slave_response(uint8_t *data, size_t max_size);
  uint8_t calculate_checksum(const uint8_t *buf, size_t len);

  // Data type helpers (from original ithowifi)
  uint32_t get_divider_from_datatype(int8_t datatype);
  uint8_t get_length_from_datatype(int8_t datatype);
  bool get_signed_from_datatype(int8_t datatype);
  int32_t cast_to_signed_int(uint32_t value, uint8_t length);

  // Custom I2C bus
  IthoI2CBus *bus_{nullptr};

  // Device state
  uint8_t remote_id_[3] = {0x00, 0x00, 0x01};
  uint8_t cmd_counter_ = 0;
  uint16_t current_speed_ = 0;

  // Sensors
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};
  sensor::Sensor *fan_speed_sensor_{nullptr};
  sensor::Sensor *fan_setpoint_sensor_{nullptr};
  sensor::Sensor *fan_speed_rpm_sensor_{nullptr};
  text_sensor::TextSensor *device_type_sensor_{nullptr};

  // Device info
  uint8_t device_group_ = 0;
  uint8_t device_id_ = 0;
  uint8_t hw_version_ = 0;
  uint8_t fw_version_ = 0;

  // Status format descriptors (from query 0x24 0x00)
  struct StatusDescriptor {
    uint8_t length;      // Number of bytes for this value
    uint32_t divider;    // Divider for float conversion
    bool is_signed;      // Whether value is signed
    bool is_float;       // Whether to treat as float (divider != 1)
  };
  std::vector<StatusDescriptor> status_format_;
  bool status_format_received_ = false;
};

// Fan platform class
class IthoI2CFan : public fan::Fan, public Component {
 public:
  IthoI2CFan(IthoI2CComponent *parent) : parent_(parent) {}

  void setup() override;
  void dump_config() override;
  fan::FanTraits get_traits() override;

 protected:
  void control(const fan::FanCall &call) override;

  IthoI2CComponent *parent_;
};

}  // namespace itho_i2c
}  // namespace esphome
