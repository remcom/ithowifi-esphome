#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32
#include <driver/i2c.h>
#endif

namespace esphome {
namespace itho_i2c {

/// Custom I2C bus component that supports both master and slave modes
/// This is needed for the Itho protocol which requires receiving responses via I2C slave mode
class IthoI2CBus : public Component {
 public:
  void set_sda_pin(uint8_t sda_pin) { sda_pin_ = sda_pin; }
  void set_scl_pin(uint8_t scl_pin) { scl_pin_ = scl_pin; }
  void set_frequency(uint32_t frequency) { frequency_ = frequency; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

  /// Write data to I2C device (master mode) - keeps master mode active
  bool write(uint8_t address, const uint8_t *data, size_t len);

  /// Write data and immediately release master - needed for query commands
  /// This allows the Itho device to respond as master to our slave
  bool write_and_release(uint8_t address, const uint8_t *data, size_t len);

  /// Read data from I2C device (master mode)
  bool read(uint8_t address, uint8_t *data, size_t len);

  /// Receive data via I2C slave mode
  /// This temporarily switches to slave mode, reads data, then switches back to master
  size_t receive_as_slave(uint8_t slave_address, uint8_t *data, size_t max_len, uint32_t timeout_ms = 200);

  /// Combined write and receive for query commands
  /// Sends command, immediately switches to slave, receives response
  /// This minimizes the gap between send and receive to avoid missing the response
  size_t write_query_and_receive(uint8_t write_address, const uint8_t *write_data, size_t write_len,
                                   uint8_t slave_address, uint8_t *read_data, size_t read_max_len,
                                   uint32_t timeout_ms = 200);

 protected:
  uint8_t sda_pin_{21};
  uint8_t scl_pin_{22};
  uint32_t frequency_{100000};

#ifdef USE_ESP32
  i2c_port_t i2c_port_{I2C_NUM_0};
  bool master_installed_{false};
  bool slave_installed_{false};
  uint8_t current_slave_address_{0x40};  // Default Itho slave address

  bool ensure_master_mode();
  void deinit_master();
  bool ensure_slave_mode();
  bool init_slave(uint8_t slave_address);
  void deinit_slave();
#endif
};

}  // namespace itho_i2c
}  // namespace esphome
