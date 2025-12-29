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

void IthoI2CComponent::query_status() {
  // Query status command from original ithowifi code
  // Full command: {0x82, 0x80, 0x31, 0xD9, 0x04, 0x00, 0xF0}
  uint8_t command[] = {I2C_ADDR_WRITE, 0x80, 0x31, 0xD9, 0x04, 0x00, 0xF0};

  ESP_LOGI(TAG, "Querying device status...");

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

    // TODO: Parse actual status values
    // The response format depends on the Itho device model
    // Common fields might include:
    // - Fan speed
    // - Temperature
    // - Humidity
    // - Operating mode
    // - Errors/warnings
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
