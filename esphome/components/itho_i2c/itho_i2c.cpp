#include "itho_i2c.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace itho_i2c {

static const char *const TAG = "itho_i2c";

// I2C addresses
static const uint8_t I2C_ADDR_WRITE = 0x82;
static const uint8_t I2C_ADDR_PWM = 0x00;

void IthoI2CComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Itho I2C...");

  // Query device type on startup
  this->set_timeout(1000, [this]() {
    this->query_device_type();
  });

  // Query status format to understand data types (once at startup)
  this->set_timeout(2000, [this]() {
    this->query_status_format();
  });

  // Set up periodic status query (every 30 seconds)
  // First query after status format is received
  this->set_timeout(3000, [this]() {
    this->query_status();
  });

  this->set_interval("status_query", 30000, [this]() {
    this->query_status();
  });
}

void IthoI2CComponent::loop() {
  // Periodic status updates can be added here if needed
}

void IthoI2CComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Itho I2C:");

  if (this->device_type_sensor_) {
    ESP_LOGCONFIG(TAG, "  Device Type: %s", this->device_type_sensor_->state.c_str());
  }

  ESP_LOGCONFIG(TAG, "  Remote ID: %02X %02X %02X",
                this->remote_id_[0], this->remote_id_[1], this->remote_id_[2]);
}

uint8_t IthoI2CComponent::calculate_checksum(const uint8_t *buf, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum += buf[i];
  }
  return 0 - sum;
}

// Data type helper functions (from original ithowifi)
uint32_t IthoI2CComponent::get_divider_from_datatype(int8_t datatype) {
  const uint32_t divider_table[] = {
    1, 10, 100, 1000, 10000, 100000,
    1000000, 10000000, 100000000,
    1, 1,  // dividers index 9 and 10 should be 0.1 and 0.01
    1, 1, 1, 256, 2
  };
  return divider_table[datatype & 0x0f];
}

uint8_t IthoI2CComponent::get_length_from_datatype(int8_t datatype) {
  switch (datatype & 0x70) {
    case 0x10:
      return 2;
    case 0x20:
    case 0x70:
      return 4;
    default:
      return 1;
  }
}

bool IthoI2CComponent::get_signed_from_datatype(int8_t datatype) {
  return datatype & 0x80;
}

int32_t IthoI2CComponent::cast_to_signed_int(uint32_t value, uint8_t length) {
  // Sign extend based on the actual data length
  if (length == 1) {
    return (int8_t)value;
  } else if (length == 2) {
    return (int16_t)value;
  } else if (length == 4) {
    return (int32_t)value;
  }
  return value;
}

bool IthoI2CComponent::send_i2c_bytes(const uint8_t *buf, size_t len, bool release_master) {
  if (len == 0) {
    ESP_LOGW(TAG, "Cannot send empty I2C command");
    return false;
  }

  ESP_LOGD(TAG, "Sending I2C bytes (%d): %s", len, format_hex_pretty(buf, len).c_str());

  // First byte is the I2C address in 8-bit format (address << 1 | write_bit)
  // ESPHome's I2C expects 7-bit address, so shift right by 1
  uint8_t address = buf[0] >> 1;

  ESP_LOGD(TAG, "Target I2C address: 0x%02X (from byte 0x%02X)", address, buf[0]);

  // Create write buffer for the command data (everything after the address byte)
  const uint8_t *data = &buf[1];
  size_t data_len = len - 1;

  // Use our custom I2C bus to write
  if (this->bus_ == nullptr) {
    ESP_LOGE(TAG, "I2C bus not configured");
    return false;
  }

  bool success;
  if (release_master) {
    // For query commands: write and immediately release master so Itho can respond
    success = this->bus_->write_and_release(address, data, data_len);
  } else {
    // For PWM commands: keep master mode active
    success = this->bus_->write(address, data, data_len);
  }

  if (!success) {
    ESP_LOGW(TAG, "I2C write to 0x%02X failed", address);
    return false;
  }

  ESP_LOGD(TAG, "I2C write to 0x%02X successful", address);
  return true;
}

bool IthoI2CComponent::send_i2c_command(const uint8_t *cmd, size_t len, bool release_master) {
  return this->send_i2c_bytes(cmd, len, release_master);
}

bool IthoI2CComponent::read_i2c_response(uint8_t addr, uint8_t *data, size_t size) {
  ESP_LOGD(TAG, "Reading %d bytes from address 0x%02X using slave mode", size, addr);

  // Use slave mode to receive response
  size_t received = this->read_i2c_slave_response(data, size);

  if (received == 0) {
    ESP_LOGW(TAG, "I2C slave mode read failed or no data received");
    return false;
  }

  ESP_LOGD(TAG, "Received %d bytes via slave mode: %s", received, format_hex_pretty(data, received).c_str());
  return true;
}

size_t IthoI2CComponent::read_i2c_slave_response(uint8_t *data, size_t max_size) {
  if (this->bus_ == nullptr) {
    ESP_LOGE(TAG, "I2C bus not configured");
    return 0;
  }

  // Use our custom I2C bus's slave mode reception
  // Slave address 0x40 per Itho protocol
  static const uint8_t I2C_SLAVE_ADDR = 0x40;

  return this->bus_->receive_as_slave(I2C_SLAVE_ADDR, data, max_size, 200);
}

void IthoI2CComponent::set_pwm_speed(uint16_t value) {
  // Command structure for PWM control
  // {0x00, 0x60, 0xC0, 0x20, 0x01, 0x02, 0xFF, 0x00, 0xFF}
  uint8_t command[9] = {
    I2C_ADDR_PWM,  // I2C address
    0x60,           // Command
    0xC0,
    0x20,
    0x01,
    0x02,
    (uint8_t)(value & 0xFF),  // Speed value (0-254)
    0x00,
    0x00
  };

  // Calculate checksum for last byte
  command[8] = this->calculate_checksum(command, 8);

  if (this->send_i2c_bytes(command, sizeof(command))) {
    this->current_speed_ = value;

    if (this->fan_speed_sensor_ != nullptr) {
      this->fan_speed_sensor_->publish_state(value);
    }

    ESP_LOGI(TAG, "Set PWM speed to %d", value);
  }
}

void IthoI2CComponent::send_remote_command(IthoCommand command, uint8_t remote_index) {
  // Remote command structure (simplified version)
  // Based on sendRemoteCmd from original code
  // [I2C addr ][  I2C command   ][len ][    timestamp         ][fmt ][    remote ID   ][cntr][opcode...][chk]

  uint8_t i2c_command[64] = {0};

  // I2C header
  i2c_command[0] = I2C_ADDR_WRITE;  // 0x82
  i2c_command[1] = 0x60;
  i2c_command[2] = 0xC1;
  i2c_command[3] = 0x01;
  i2c_command[4] = 0x01;
  i2c_command[5] = 0x09;  // Length

  // Timestamp (use millis())
  uint32_t timestamp = millis();
  i2c_command[6] = (timestamp >> 24) & 0xFF;
  i2c_command[7] = (timestamp >> 16) & 0xFF;
  i2c_command[8] = (timestamp >> 8) & 0xFF;
  i2c_command[9] = timestamp & 0xFF;

  // Format
  i2c_command[10] = 0x16;

  // Remote ID
  i2c_command[11] = this->remote_id_[0];
  i2c_command[12] = this->remote_id_[1];
  i2c_command[13] = this->remote_id_[2];

  // Counter
  i2c_command[14] = this->cmd_counter_++;

  // Command opcodes (simplified - would need full remote command data)
  // This is a placeholder - you would need to implement the full remote command structure
  // based on the RemoteTypes and command data from the original code

  uint8_t cmd_len = 15;  // Base length, would be extended with actual command data

  // Checksum
  i2c_command[cmd_len] = this->calculate_checksum(i2c_command, cmd_len);
  cmd_len++;

  if (this->send_i2c_bytes(i2c_command, cmd_len)) {
    ESP_LOGI(TAG, "Sent remote command: %d", command);
  }
}

void IthoI2CComponent::query_device_type() {
  // Query device type command from original ithowifi code
  // Full command: {0x82, 0x80, 0x90, 0xE0, 0x04, 0x00, 0x8A}
  uint8_t command[] = {I2C_ADDR_WRITE, 0x80, 0x90, 0xE0, 0x04, 0x00, 0x8A};

  ESP_LOGI(TAG, "Querying device type...");

  if (this->bus_ == nullptr) {
    ESP_LOGE(TAG, "I2C bus not configured");
    return;
  }

  // Extract address and data from command
  uint8_t address = command[0] >> 1;  // 0x82 >> 1 = 0x41
  const uint8_t *data = &command[1];
  size_t data_len = sizeof(command) - 1;

  // Use combined write+receive to minimize timing gap
  uint8_t response[64] = {0};
  static const uint8_t I2C_SLAVE_ADDR = 0x40;

  size_t len = this->bus_->write_query_and_receive(address, data, data_len,
                                                     I2C_SLAVE_ADDR, response, sizeof(response), 200);

  if (len > 0) {
    // Parse device type from response
    // Response format: bytes 8,9,10 contain device group, ID, and HW version
    ESP_LOGD(TAG, "Device type response received");
    ESP_LOGI(TAG, "Raw device type data: %s", format_hex_pretty(response, 32).c_str());

    if (response[0] == 0x80 && response[1] == 0x82) {
      this->device_group_ = response[8];
      this->device_id_ = response[9];
      this->hw_version_ = response[10];

      ESP_LOGI(TAG, "Device: Group=0x%02X, ID=0x%02X, HW=0x%02X",
               this->device_group_, this->device_id_, this->hw_version_);

      if (this->device_type_sensor_ != nullptr) {
        // This would need proper parsing from device tables
        char device_str[32];
        snprintf(device_str, sizeof(device_str), "Itho 0x%02X/0x%02X",
                 this->device_group_, this->device_id_);
        this->device_type_sensor_->publish_state(device_str);
      }
    }
  } else {
    ESP_LOGW(TAG, "No response received for device type query");
  }
}

void IthoI2CComponent::query_status_format() {
  // Query status FORMAT command - 0x24 0x00
  // Returns data type descriptors for each status value
  // Full command: {0x82, 0x80, 0x24, 0x00, 0x04, 0x00, 0xD6}
  uint8_t command[] = {I2C_ADDR_WRITE, 0x80, 0x24, 0x00, 0x04, 0x00, 0xD6};

  ESP_LOGI(TAG, "Querying device status format (0x24 0x00)...");

  if (this->bus_ == nullptr) {
    ESP_LOGE(TAG, "I2C bus not configured");
    return;
  }

  // Extract address and data from command
  uint8_t address = command[0] >> 1;  // 0x82 >> 1 = 0x41
  const uint8_t *data = &command[1];
  size_t data_len = sizeof(command) - 1;

  // Use combined write+receive to minimize timing gap
  uint8_t response[64] = {0};
  static const uint8_t I2C_SLAVE_ADDR = 0x40;

  size_t len = this->bus_->write_query_and_receive(address, data, data_len,
                                                     I2C_SLAVE_ADDR, response, sizeof(response), 200);

  if (len > 0) {
    ESP_LOGI(TAG, "Status format response received (%d bytes)", len);
    ESP_LOGI(TAG, "Raw format data: %s", format_hex_pretty(response, len).c_str());

    // Parse status format response
    // Format: [addr][0x82][0xA4][0x00][type][count][datatype1][datatype2]...[checksum]
    // Example: 80 82 A4 00 01 0C 80 10 10 10 00 10 20 10 10 00 92 92 29
    if (len >= 7 && response[1] == 0x82 && response[2] == 0xA4 && response[3] == 0x00) {
      uint8_t num_descriptors = response[5];
      ESP_LOGI(TAG, "Status format: %d value descriptors", num_descriptors);

      // Clear previous format
      this->status_format_.clear();

      // Parse each data type descriptor
      for (uint8_t i = 0; i < num_descriptors && (6 + i) < (len - 1); i++) {
        int8_t datatype = response[6 + i];

        StatusDescriptor desc;
        desc.length = this->get_length_from_datatype(datatype);
        desc.divider = this->get_divider_from_datatype(datatype);
        desc.is_signed = this->get_signed_from_datatype(datatype);
        desc.is_float = (desc.divider != 1);

        // Special case for legacy 0x5B datatype
        if (datatype == 0x5B) {
          desc.is_float = false;
          desc.length = 2;
          desc.is_signed = false;
        }

        this->status_format_.push_back(desc);

        ESP_LOGD(TAG, "  Value %d: datatype=0x%02X, len=%d, div=%u, signed=%d, float=%d",
                 i, datatype, desc.length, desc.divider, desc.is_signed, desc.is_float);
      }

      this->status_format_received_ = true;
      ESP_LOGI(TAG, "Status format received successfully");
    } else if (len >= 3) {
      ESP_LOGW(TAG, "Unexpected format response header: %02X %02X %02X",
               response[1], response[2], response[3]);
    } else {
      ESP_LOGW(TAG, "Status format response too short (%d bytes)", len);
    }
  } else {
    ESP_LOGW(TAG, "No response received for status format query");
  }
}

void IthoI2CComponent::query_status() {
  // Query status VALUES command - 0x24 0x01
  // This is the CORRECT command for CVE1B status (includes temp/humidity/rpm/etc)
  // Full command: {0x82, 0x80, 0x24, 0x01, 0x04, 0x00, 0xD5}
  uint8_t command[] = {I2C_ADDR_WRITE, 0x80, 0x24, 0x01, 0x04, 0x00, 0xD5};

  ESP_LOGI(TAG, "Querying device status (0x24 0x01)...");

  if (this->bus_ == nullptr) {
    ESP_LOGE(TAG, "I2C bus not configured");
    return;
  }

  // Extract address and data from command
  uint8_t address = command[0] >> 1;  // 0x82 >> 1 = 0x41
  const uint8_t *data = &command[1];
  size_t data_len = sizeof(command) - 1;

  // Use combined write+receive to minimize timing gap
  uint8_t response[64] = {0};
  static const uint8_t I2C_SLAVE_ADDR = 0x40;

  size_t len = this->bus_->write_query_and_receive(address, data, data_len,
                                                     I2C_SLAVE_ADDR, response, sizeof(response), 200);

  if (len > 0) {
    ESP_LOGD(TAG, "Status response received (%d bytes)", len);
    ESP_LOGI(TAG, "Raw status data: %s", format_hex_pretty(response, len).c_str());

    // Parse status response - CVE1B format
    // Format: [addr][0x82][0xA4][0x01][type][count][data...][checksum]
    // Example: 80.82.A4.01.01.17.FF.03.A5.03.A7...
    if (len >= 7 && response[1] == 0x82 && response[2] == 0xA4 && response[3] == 0x01) {
      uint8_t num_values = response[5];  // 0x17 = 23 values

      ESP_LOGI(TAG, "CVE1B Status: %d values received", num_values);

      // Check if we have status format information
      if (!this->status_format_received_ || this->status_format_.empty()) {
        ESP_LOGW(TAG, "Status format not yet received, skipping detailed parsing");
        return;
      }

      // CVE1B status labels (from original ithowifi):
      // 0: Ventilation setpoint (%), 1: Fan setpoint (rpm), 2: Fan speed (rpm)
      // 23: Internal humidity (%), 24: Internal temp (°C)
      // 33: RelativeHumidity, 34: Temperature

      size_t data_pos = 6;  // Data starts at byte 6

      // Parse each value according to its descriptor
      for (uint8_t i = 0; i < num_values && i < this->status_format_.size(); i++) {
        const StatusDescriptor &desc = this->status_format_[i];

        // Check we have enough bytes
        if (data_pos + desc.length > len - 1) {
          ESP_LOGW(TAG, "Not enough bytes for value %d (need %d, have %d)",
                   i, desc.length, len - 1 - data_pos);
          break;
        }

        // Read the raw value (big-endian)
        uint32_t raw_value = 0;
        for (uint8_t j = 0; j < desc.length; j++) {
          raw_value |= response[data_pos + (desc.length - 1 - j)] << (j * 8);
        }

        // Convert to signed if needed
        int32_t signed_value = raw_value;
        if (desc.is_signed) {
          signed_value = this->cast_to_signed_int(raw_value, desc.length);
        }

        // Convert to float if needed
        float float_value = static_cast<float>(signed_value) / desc.divider;

        // Log and publish specific values
        if (i == 0) {
          ESP_LOGI(TAG, "  [0] Ventilation setpoint: %d%%", signed_value);
        } else if (i == 1) {
          ESP_LOGI(TAG, "  [1] Fan setpoint: %d rpm", signed_value);
          // Publish fan setpoint sensor
          if (this->fan_setpoint_sensor_ != nullptr && signed_value > 0) {
            this->fan_setpoint_sensor_->publish_state(signed_value);
          }
        } else if (i == 2) {
          ESP_LOGI(TAG, "  [2] Fan speed: %d rpm", signed_value);

          // Publish fan speed RPM sensor
          if (this->fan_speed_rpm_sensor_ != nullptr && signed_value > 0) {
            this->fan_speed_rpm_sensor_->publish_state(signed_value);
          }

          // Publish fan speed percentage sensor
          if (this->fan_speed_sensor_ != nullptr && signed_value > 0) {
            // Convert RPM to percentage (assuming ~1200 RPM = 100%)
            float speed_percent = (signed_value * 100.0f) / 1200.0f;
            if (speed_percent > 100.0f) speed_percent = 100.0f;
            this->fan_speed_sensor_->publish_state(speed_percent);
          }
        } else if (i == 10) {
          // Humidity (index 10 for your device)
          if (desc.is_float) {
            ESP_LOGI(TAG, "  [10] Humidity: %.1f%%", float_value);
            if (this->humidity_sensor_ != nullptr && float_value >= 0 && float_value <= 100) {
              this->humidity_sensor_->publish_state(float_value);
            }
          } else {
            ESP_LOGI(TAG, "  [10] Humidity: %d%%", signed_value);
            if (this->humidity_sensor_ != nullptr && signed_value >= 0 && signed_value <= 100) {
              this->humidity_sensor_->publish_state(signed_value);
            }
          }
        } else if (i == 11) {
          // Temperature (index 11 for your device)
          if (desc.is_float) {
            ESP_LOGI(TAG, "  [11] Temperature: %.1f°C", float_value);
            if (this->temperature_sensor_ != nullptr && float_value > -50 && float_value < 100) {
              this->temperature_sensor_->publish_state(float_value);
            }
          } else {
            ESP_LOGI(TAG, "  [11] Temperature: %d°C", signed_value);
            if (this->temperature_sensor_ != nullptr && signed_value > -50 && signed_value < 100) {
              this->temperature_sensor_->publish_state(signed_value);
            }
          }
        } else {
          // Log other values at debug level
          if (desc.is_float) {
            ESP_LOGD(TAG, "  [%d] Value: %.2f (raw=0x%X)", i, float_value, raw_value);
          } else {
            ESP_LOGD(TAG, "  [%d] Value: %d (raw=0x%X)", i, signed_value, raw_value);
          }
        }

        data_pos += desc.length;
      }

      ESP_LOGI(TAG, "Status parsing complete");

    } else if (len >= 3) {
      ESP_LOGW(TAG, "Unexpected response header: %02X %02X %02X",
               response[1], response[2], response[3]);
    } else {
      ESP_LOGW(TAG, "Status response too short (%d bytes)", len);
    }
  } else {
    ESP_LOGW(TAG, "No response received for status query");
  }
}

void IthoI2CComponent::query_counters() {
  // Query counters command from original ithowifi code
  // Full command: {0x82, 0x80, 0x42, 0x10, 0x04, 0x00, 0xA8}
  uint8_t command[] = {I2C_ADDR_WRITE, 0x80, 0x42, 0x10, 0x04, 0x00, 0xA8};

  ESP_LOGI(TAG, "Querying device counters...");

  if (this->bus_ == nullptr) {
    ESP_LOGE(TAG, "I2C bus not configured");
    return;
  }

  // Extract address and data from command
  uint8_t address = command[0] >> 1;  // 0x82 >> 1 = 0x41
  const uint8_t *data = &command[1];
  size_t data_len = sizeof(command) - 1;

  // Use combined write+receive to minimize timing gap
  uint8_t response[64] = {0};
  static const uint8_t I2C_SLAVE_ADDR = 0x40;

  size_t len = this->bus_->write_query_and_receive(address, data, data_len,
                                                     I2C_SLAVE_ADDR, response, sizeof(response), 200);

  if (len > 0) {
    ESP_LOGD(TAG, "Counters response received (%d bytes)", len);
    ESP_LOGI(TAG, "Raw counters data: %s", format_hex_pretty(response, len).c_str());

    // TODO: Parse actual counter values
    // Common counters include:
    // - Filter hours remaining
    // - Total operating hours
    // - Error counters
  } else {
    ESP_LOGW(TAG, "No response received for counters query");
  }
}

void IthoI2CComponent::query_measurements() {
  // Query measurements command - 0x12 query type
  // This may return temperature, humidity, and other sensor values
  // Full command: {0x82, 0x80, 0x12, 0xXX, 0x04, 0x00, checksum}
  uint8_t command[] = {I2C_ADDR_WRITE, 0x80, 0x12, 0xA4, 0x04, 0x00, 0x00};

  // Calculate checksum
  command[6] = this->calculate_checksum(command, 6);

  ESP_LOGI(TAG, "Querying device measurements...");

  if (this->bus_ == nullptr) {
    ESP_LOGE(TAG, "I2C bus not configured");
    return;
  }

  // Extract address and data from command
  uint8_t address = command[0] >> 1;  // 0x82 >> 1 = 0x41
  const uint8_t *data = &command[1];
  size_t data_len = sizeof(command) - 1;

  // Use combined write+receive to minimize timing gap
  uint8_t response[64] = {0};
  static const uint8_t I2C_SLAVE_ADDR = 0x40;

  size_t len = this->bus_->write_query_and_receive(address, data, data_len,
                                                     I2C_SLAVE_ADDR, response, sizeof(response), 200);

  if (len > 0) {
    ESP_LOGD(TAG, "Measurements response received (%d bytes)", len);
    ESP_LOGI(TAG, "Raw measurements data: %s", format_hex_pretty(response, len).c_str());

    // Parse measurements response
    if (len >= 10 && response[1] == 0x82) {
      // Try to extract temperature and humidity
      // Format may vary by device model

      // Log all non-padding bytes for analysis
      ESP_LOGI(TAG, "Analyzing measurement bytes:");
      for (size_t i = 6; i < len - 1 && i < 20; i++) {
        if (response[i] != 0x20 && response[i] != 0x00) {
          ESP_LOGI(TAG, "  Byte %d: 0x%02X (%d)", i, response[i], response[i]);
        }
      }
    }
  } else {
    ESP_LOGW(TAG, "No response received for measurements query");
  }
}

void IthoI2CComponent::set_speed(uint8_t speed) {
  // Convert speed percentage to PWM value (0-254)
  uint16_t pwm_value = (speed * 254) / 100;
  this->set_pwm_speed(pwm_value);
}

// IthoI2CFan implementation
void IthoI2CFan::setup() {
  // Initialize fan state
  this->state = false;
  this->speed = 0;
}

void IthoI2CFan::dump_config() {
  ESP_LOGCONFIG(TAG, "Itho I2C Fan");
}

fan::FanTraits IthoI2CFan::get_traits() {
  return fan::FanTraits(false, true, false, 100);  // oscillation=false, speed=true, direction=false, speed_count=100
}

void IthoI2CFan::control(const fan::FanCall &call) {
  if (call.get_state().has_value()) {
    this->state = *call.get_state();

    if (!this->state) {
      // Turn off - set speed to 0
      this->parent_->set_speed(0);
      this->speed = 0;
    }
  }

  if (call.get_speed().has_value()) {
    this->speed = *call.get_speed();

    if (this->speed > 0) {
      this->state = true;
      this->parent_->set_speed(this->speed);
    } else {
      this->state = false;
      this->parent_->set_speed(0);
    }
  }

  this->publish_state();
}

}  // namespace itho_i2c
}  // namespace esphome
