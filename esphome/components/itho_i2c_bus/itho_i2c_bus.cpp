#include "itho_i2c_bus.h"
#include "esphome/core/log.h"

namespace esphome {
namespace itho_i2c {

static const char *const TAG = "itho_i2c_bus";

void IthoI2CBus::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Itho I2C Bus...");
  ESP_LOGCONFIG(TAG, "  SDA Pin: GPIO%u", this->sda_pin_);
  ESP_LOGCONFIG(TAG, "  SCL Pin: GPIO%u", this->scl_pin_);
  ESP_LOGCONFIG(TAG, "  Frequency: %u Hz", this->frequency_);

#ifdef USE_ESP32
  ESP_LOGCONFIG(TAG, "  I2C Port: %d", this->i2c_port_);

  // Start in MASTER mode - simpler, standard I2C
  ESP_LOGI(TAG, "Starting in I2C master mode");
  if (!this->ensure_master_mode()) {
    ESP_LOGE(TAG, "Failed to initialize I2C master mode");
    this->mark_failed();
    return;
  }
#endif

  ESP_LOGCONFIG(TAG, "Itho I2C Bus setup complete");
}

void IthoI2CBus::dump_config() {
  ESP_LOGCONFIG(TAG, "Itho I2C Bus:");
  ESP_LOGCONFIG(TAG, "  SDA Pin: GPIO%u", this->sda_pin_);
  ESP_LOGCONFIG(TAG, "  SCL Pin: GPIO%u", this->scl_pin_);
  ESP_LOGCONFIG(TAG, "  Frequency: %u Hz", this->frequency_);
#ifdef USE_ESP32
  ESP_LOGCONFIG(TAG, "  I2C Port: %d", this->i2c_port_);
#endif
}

#ifdef USE_ESP32

bool IthoI2CBus::ensure_master_mode() {
  if (this->master_installed_) {
    return true;
  }

  ESP_LOGD(TAG, "Initializing I2C master mode");

  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)this->sda_pin_;
  conf.scl_io_num = (gpio_num_t)this->scl_pin_;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = this->frequency_;
  conf.clk_flags = 0;

  esp_err_t err = i2c_param_config(this->i2c_port_, &conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C master param config failed: %d", err);
    return false;
  }

  err = i2c_driver_install(this->i2c_port_, conf.mode, 0, 0, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C master driver install failed: %d", err);
    return false;
  }

  // Set I2C timeout to a reasonable value (default is often too short)
  err = i2c_set_timeout(this->i2c_port_, 0xFFFFF);  // Max timeout
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to set I2C timeout: %d", err);
  }

  this->master_installed_ = true;
  ESP_LOGD(TAG, "I2C master mode initialized successfully");
  return true;
}

void IthoI2CBus::deinit_master() {
  if (!this->master_installed_) {
    return;
  }

  ESP_LOGD(TAG, "Deinitializing I2C master mode");
  i2c_driver_delete(this->i2c_port_);
  this->master_installed_ = false;
}

bool IthoI2CBus::init_slave(uint8_t slave_address) {
  ESP_LOGD(TAG, "Initializing I2C slave mode at address 0x%02X", slave_address);

  i2c_config_t conf_slave = {};
  conf_slave.mode = I2C_MODE_SLAVE;
  conf_slave.sda_io_num = (gpio_num_t)this->sda_pin_;
  conf_slave.scl_io_num = (gpio_num_t)this->scl_pin_;
  conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf_slave.slave.addr_10bit_en = 0;
  conf_slave.slave.slave_addr = slave_address;
  conf_slave.slave.maximum_speed = 400000;  // 400kHz as per original ithowifi code
  conf_slave.clk_flags = 0;

  esp_err_t err = i2c_param_config(this->i2c_port_, &conf_slave);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C slave param config failed: %d", err);
    return false;
  }

  // RX buffer size 128, TX buffer size 0 (we don't transmit)
  err = i2c_driver_install(this->i2c_port_, conf_slave.mode, 128, 0, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C slave driver install failed: %d", err);
    return false;
  }

  // Set slave timeout
  err = i2c_set_timeout(this->i2c_port_, 0xFFFFF);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to set I2C slave timeout: %d", err);
  }

  ESP_LOGD(TAG, "I2C slave mode initialized");
  return true;
}

void IthoI2CBus::deinit_slave() {
  ESP_LOGD(TAG, "Deinitializing I2C slave mode");
  i2c_driver_delete(this->i2c_port_);
  this->slave_installed_ = false;
}

bool IthoI2CBus::ensure_slave_mode() {
  if (this->slave_installed_) {
    return true;  // Already in slave mode
  }

  ESP_LOGD(TAG, "Ensuring I2C slave mode at address 0x%02X", this->current_slave_address_);

  // Deinit master if active
  if (this->master_installed_) {
    this->deinit_master();
  }

  // Initialize slave
  if (!this->init_slave(this->current_slave_address_)) {
    return false;
  }

  this->slave_installed_ = true;
  return true;
}

#endif  // USE_ESP32

bool IthoI2CBus::write(uint8_t address, const uint8_t *data, size_t len) {
#ifdef USE_ESP32
  if (!this->ensure_master_mode()) {
    ESP_LOGE(TAG, "Master mode not initialized");
    return false;
  }

  ESP_LOGD(TAG, "Writing %d bytes to address 0x%02X", len, address);
  ESP_LOGV(TAG, "Data: %s", format_hex_pretty(data, len).c_str());

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write(cmd, data, len, true);
  i2c_master_stop(cmd);

  // Increased timeout to 2 seconds for slow devices
  esp_err_t err = i2c_master_cmd_begin(this->i2c_port_, cmd, pdMS_TO_TICKS(2000));
  i2c_cmd_link_delete(cmd);

  if (err != ESP_OK) {
    const char *err_name = "UNKNOWN";
    if (err == ESP_ERR_TIMEOUT) err_name = "TIMEOUT";
    else if (err == ESP_FAIL) err_name = "NACK";
    else if (err == ESP_ERR_INVALID_STATE) err_name = "INVALID_STATE";

    ESP_LOGW(TAG, "I2C write to 0x%02X failed: %s (0x%X)", address, err_name, err);
    return false;
  }

  ESP_LOGD(TAG, "I2C write to 0x%02X successful", address);
  return true;
#else
  return false;
#endif
}

bool IthoI2CBus::write_and_release(uint8_t address, const uint8_t *data, size_t len) {
#ifdef USE_ESP32
  if (!this->ensure_master_mode()) {
    ESP_LOGE(TAG, "Master mode not initialized");
    return false;
  }

  ESP_LOGD(TAG, "Writing %d bytes to address 0x%02X (with release)", len, address);
  ESP_LOGV(TAG, "Data: %s", format_hex_pretty(data, len).c_str());

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write(cmd, data, len, true);
  i2c_master_stop(cmd);

  esp_err_t err = i2c_master_cmd_begin(this->i2c_port_, cmd, pdMS_TO_TICKS(200));
  i2c_cmd_link_delete(cmd);

  if (err != ESP_OK) {
    const char *err_name = "UNKNOWN";
    if (err == ESP_ERR_TIMEOUT) err_name = "TIMEOUT";
    else if (err == ESP_FAIL) err_name = "NACK";
    else if (err == ESP_ERR_INVALID_STATE) err_name = "INVALID_STATE";

    ESP_LOGW(TAG, "I2C write to 0x%02X failed: %s (0x%X)", address, err_name, err);
    return false;
  }

  // CRITICAL: Deinitialize master immediately to allow Itho to respond
  ESP_LOGD(TAG, "I2C write successful, releasing master mode");
  this->deinit_master();

  return true;
#else
  return false;
#endif
}

bool IthoI2CBus::write_raw(const uint8_t *data, size_t len) {
#ifdef USE_ESP32
  if (!this->ensure_master_mode()) {
    ESP_LOGE(TAG, "Master mode not initialized");
    return false;
  }

  ESP_LOGD(TAG, "Writing %d raw bytes to I2C bus", len);
  ESP_LOGV(TAG, "Raw data: %s", format_hex_pretty(data, len).c_str());

  // Like original i2c_master_send - write entire buffer as-is
  // The first byte (0x82) is part of the I2C protocol on the wire
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write(cmd, data, len, true);
  i2c_master_stop(cmd);

  esp_err_t err = i2c_master_cmd_begin(this->i2c_port_, cmd, pdMS_TO_TICKS(200));
  i2c_cmd_link_delete(cmd);

  if (err != ESP_OK) {
    const char *err_name = "UNKNOWN";
    if (err == ESP_ERR_TIMEOUT) err_name = "TIMEOUT";
    else if (err == ESP_FAIL) err_name = "NACK";
    else if (err == ESP_ERR_INVALID_STATE) err_name = "INVALID_STATE";

    ESP_LOGW(TAG, "I2C raw write failed: %s (0x%X)", err_name, err);
    return false;
  }

  // Deinitialize master after sending remote command
  ESP_LOGD(TAG, "I2C raw write successful, releasing master mode");
  this->deinit_master();

  return true;
#else
  return false;
#endif
}

bool IthoI2CBus::read(uint8_t address, uint8_t *data, size_t len) {
#ifdef USE_ESP32
  if (!this->ensure_master_mode()) {
    return false;
  }

  ESP_LOGD(TAG, "Reading %d bytes from address 0x%02X", len, address);

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
  if (len > 1) {
    i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);

  esp_err_t err = i2c_master_cmd_begin(this->i2c_port_, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);

  if (err != ESP_OK) {
    ESP_LOGW(TAG, "I2C read from 0x%02X failed: %d", address, err);
    return false;
  }

  ESP_LOGD(TAG, "I2C read from 0x%02X successful", address);
  return true;
#else
  return false;
#endif
}

size_t IthoI2CBus::receive_as_slave(uint8_t slave_address, uint8_t *data, size_t max_len, uint32_t timeout_ms) {
#ifdef USE_ESP32
  // NOTE: This function is kept for compatibility but is not the recommended approach
  // Use write_query_and_receive() instead which matches the original ithowifi implementation

  ESP_LOGD(TAG, "Switching to slave mode to receive data");

  // Deinitialize master mode
  this->deinit_master();

  // Initialize slave mode (matches original i2c_slave_receive())
  i2c_config_t conf_slave = {};
  conf_slave.mode = I2C_MODE_SLAVE;
  conf_slave.sda_io_num = (gpio_num_t)this->sda_pin_;
  conf_slave.scl_io_num = (gpio_num_t)this->scl_pin_;
  conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf_slave.slave.addr_10bit_en = 0;
  conf_slave.slave.slave_addr = slave_address;
  conf_slave.slave.maximum_speed = 400000;
  conf_slave.clk_flags = 0;

  esp_err_t err = i2c_param_config(this->i2c_port_, &conf_slave);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Slave param config failed: %d", err);
    this->ensure_master_mode();
    return 0;
  }

  err = i2c_driver_install(this->i2c_port_, conf_slave.mode, 128, 0, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Slave driver install failed: %d", err);
    this->ensure_master_mode();
    return 0;
  }

  ESP_LOGD(TAG, "Waiting for I2C slave data (timeout: %ums)...", timeout_ms);

  // Store slave address in first byte
  data[0] = slave_address << 1;

  // Read from slave RX buffer
  int len_rec = i2c_slave_read_buffer(this->i2c_port_, &data[1], max_len - 1, pdMS_TO_TICKS(timeout_ms));

  // Deinitialize slave mode
  i2c_driver_delete(this->i2c_port_);
  this->slave_installed_ = false;

  // Restore master mode
  this->ensure_master_mode();

  if (len_rec <= 0) {
    ESP_LOGD(TAG, "No data received in slave mode (timeout or error: %d)", len_rec);
    return 0;
  }

  ESP_LOGI(TAG, "Received %d bytes via I2C slave mode", len_rec + 1);
  return len_rec + 1;
#else
  ESP_LOGW(TAG, "I2C slave mode only supported on ESP32");
  return 0;
#endif
}

size_t IthoI2CBus::write_query_and_receive(uint8_t write_address, const uint8_t *write_data, size_t write_len,
                                             uint8_t slave_address, uint8_t *read_data, size_t read_max_len,
                                             uint32_t timeout_ms) {
#ifdef USE_ESP32
  // MATCHES ORIGINAL ITHOWIFI: i2c_master_send() + i2c_slave_receive()
  // This is the EXACT pattern from send_i2c_query() in IthoSystem.cpp

  // STEP 1: Send command as master (matches i2c_master_send())
  if (!this->ensure_master_mode()) {
    ESP_LOGE(TAG, "Master mode not initialized");
    return 0;
  }

  ESP_LOGI(TAG, "Sending query to 0x%02X (%d bytes): %02X %02X %02X %02X %02X %02X",
           write_address, write_len,
           write_data[0], write_data[1], write_data[2],
           write_data[3], write_data[4], write_data[5]);

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (write_address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write(cmd, write_data, write_len, true);
  i2c_master_stop(cmd);

  esp_err_t err = i2c_master_cmd_begin(this->i2c_port_, cmd, pdMS_TO_TICKS(200));
  i2c_cmd_link_delete(cmd);

  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Query write failed: %d", err);
    return 0;
  }

  ESP_LOGI(TAG, "Query sent successfully - Itho ACKed");

  // CRITICAL: Deinit master IMMEDIATELY (matches i2c_master_deinit() in original)
  ESP_LOGD(TAG, "Deinitializing master to allow Itho to respond");
  this->deinit_master();

  // STEP 2: IMMEDIATELY switch to slave mode (matches i2c_slave_receive())
  // NO DELAY - this is critical!

  ESP_LOGD(TAG, "Initializing slave mode at address 0x%02X", slave_address);

  // Configure slave mode (matches i2c_slave_receive() setup)
  i2c_config_t conf_slave = {};
  conf_slave.mode = I2C_MODE_SLAVE;
  conf_slave.sda_io_num = (gpio_num_t)this->sda_pin_;
  conf_slave.scl_io_num = (gpio_num_t)this->scl_pin_;
  conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf_slave.slave.addr_10bit_en = 0;
  conf_slave.slave.slave_addr = slave_address;
  conf_slave.slave.maximum_speed = 400000;  // 400kHz max as per original
  conf_slave.clk_flags = 0;

  err = i2c_param_config(this->i2c_port_, &conf_slave);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Slave param config failed: %d", err);
    // Restore master before returning
    this->ensure_master_mode();
    return 0;
  }

  // Install slave driver with RX buffer (matches original: 128 bytes)
  err = i2c_driver_install(this->i2c_port_, conf_slave.mode, 128, 0, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Slave driver install failed: %d", err);
    // Restore master before returning
    this->ensure_master_mode();
    return 0;
  }

  ESP_LOGD(TAG, "Slave mode initialized, waiting for data...");

  // STEP 3: Read from slave buffer (EXACTLY as original does)
  // Store slave address in first byte (matches original protocol)
  read_data[0] = slave_address << 1;

  // Read with timeout (matches original: 200ms)
  int len_rec = i2c_slave_read_buffer(this->i2c_port_, &read_data[1], read_max_len - 1,
                                       pdMS_TO_TICKS(timeout_ms));

  ESP_LOGD(TAG, "Slave read returned: %d bytes", len_rec);

  // STEP 4: Deinit slave (matches i2c_slave_deinit() in original)
  i2c_driver_delete(this->i2c_port_);
  this->slave_installed_ = false;

  // STEP 5: Restore master mode for subsequent commands
  if (!this->ensure_master_mode()) {
    ESP_LOGW(TAG, "Failed to restore master mode after slave read");
  }

  if (len_rec <= 0) {
    ESP_LOGW(TAG, "No data received in slave mode (timeout or error: %d)", len_rec);
    return 0;
  }

  ESP_LOGI(TAG, "Received %d bytes via I2C slave mode", len_rec + 1);
  ESP_LOGD(TAG, "Data: %s", format_hex_pretty(read_data, len_rec + 1).c_str());

  return len_rec + 1;  // Include address byte
#else
  ESP_LOGW(TAG, "Combined write/receive only supported on ESP32");
  return 0;
#endif
}

}  // namespace itho_i2c
}  // namespace esphome
