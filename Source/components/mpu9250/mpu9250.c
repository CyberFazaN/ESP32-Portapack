/*****************************************************************************
 *                                                                           *
 *  Copyright 2018 Simon M. Werner                                           *
 *                                                                           *
 *  Licensed under the Apache License, Version 2.0 (the "License");          *
 *  you may not use this file except in compliance with the License.         *
 *  You may obtain a copy of the License at                                  *
 *                                                                           *
 *      http://www.apache.org/licenses/LICENSE-2.0                           *
 *                                                                           *
 *  Unless required by applicable law or agreed to in writing, software      *
 *  distributed under the License is distributed on an "AS IS" BASIS,        *
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
 *  See the License for the specific language governing permissions and      *
 *  limitations under the License.                                           *
 *                                                                           *
 *****************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_idf_lib_helpers.h"

#include "mpu9250.h"
#include "ak8963.h"
#include "i2cdev.h"

static const char *TAG = "mpu9250";

static bool initialised = false;
static calibration_t *cal;

static float gyro_inv_scale = 1.0;
static float accel_inv_scale = 1.0;

typedef struct
{
  uint8_t x;
  uint8_t y;
  uint8_t z;
} power_settings_e;

static esp_err_t mpu9250_enable_magnetometer(i2c_dev_t *mpu9250_dev, i2c_dev_t *ak8963_dev);

esp_err_t i2c_write_bit(i2c_dev_t *dev, uint8_t reg, uint8_t bit, uint8_t value) {
    uint8_t current_value;

    // Read the current register value
    ESP_ERROR_CHECK(i2c_dev_read_reg(dev, reg, &current_value, 1));

    // Modify the specific bit
    if (value) {
        current_value |= (1 << bit);  // Set the bit
    } else {
        current_value &= ~(1 << bit);  // Clear the bit
    }

    // Write the modified register value back
    return i2c_dev_write_reg(dev, reg, &current_value, 1);
}

esp_err_t i2c_write_bits(i2c_dev_t *dev, uint8_t reg, uint8_t bit_start, uint8_t length, uint8_t value) {
    uint8_t current_value;

    // Read the current register value
    ESP_ERROR_CHECK(i2c_dev_read_reg(dev, reg, &current_value, 1));

    // Create a mask for the bits we want to modify
    uint8_t mask = ((1 << length) - 1) << bit_start;

    // Clear the bits we want to modify
    current_value &= ~mask;

    // Set the new value for the bits
    current_value |= (value << bit_start) & mask;

    // Write the modified register value back
    return i2c_dev_write_reg(dev, reg, &current_value, 1);
}

esp_err_t i2c_write_byte(i2c_dev_t *dev, uint8_t reg, uint8_t value) {
    // Write a single byte to the specified register
    return i2c_dev_write_reg(dev, reg, &value, 1);
}

esp_err_t i2c_read_byte(i2c_dev_t *dev, uint8_t reg, uint8_t *value) {
    // Read a single byte from the specified register
    return i2c_dev_read_reg(dev, reg, value, 1);
}

esp_err_t i2c_read_bytes(i2c_dev_t *dev, uint8_t reg, uint8_t *data, size_t length) {
    // Read multiple bytes starting from the specified register
    return i2c_dev_read_reg(dev, reg, data, length);
}

esp_err_t i2c_read_bit(i2c_dev_t *dev, uint8_t reg, uint8_t bit, uint8_t *value) {
    uint8_t current_value;

    // Read the register value
    ESP_ERROR_CHECK(i2c_dev_read_reg(dev, reg, &current_value, 1));

    // Extract the specific bit
    *value = (current_value & (1 << bit)) != 0;

    return ESP_OK;
}



esp_err_t mpu9250_init_desc(i2c_dev_t *mpu9250_dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(mpu9250_dev);

    if (addr != 0x68)
    {
        return ESP_ERR_INVALID_ARG;
    }

    mpu9250_dev->port = port;
    mpu9250_dev->addr = addr;
    mpu9250_dev->cfg.sda_io_num = sda_gpio;
    mpu9250_dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    mpu9250_dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(mpu9250_dev);
}

esp_err_t mpu9250_free_desc(i2c_dev_t *mpu9250_dev)
{
    CHECK_ARG(mpu9250_dev);
    return i2c_dev_delete_mutex(mpu9250_dev);
}

esp_err_t mpu9250_init(i2c_dev_t *mpu9250_dev, calibration_t *c)
{
    CHECK_ARG(mpu9250_dev);
    uint8_t device_id;
    if (mpu9250_get_device_id(mpu9250_dev, &device_id) != ESP_OK || device_id != 0x71 || device_id != 0x73 || device_id != 0x70)
        return ESP_FAIL;
  
    ESP_LOGI(TAG, "Initializating MPU9250");
    vTaskDelay(100 / portTICK_PERIOD_MS);

    if (initialised)
  {
    ESP_LOGE(TAG, "i2c_mpu9250_init has already been called");
    return ESP_ERR_INVALID_STATE;
  }
  initialised = true;
  cal = c;

  ESP_LOGD(TAG, "i2c_mpu9250_init");

  ESP_ERROR_CHECK(i2c_write_bit(mpu9250_dev, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_DEVICE_RESET_BIT, 1));
  vTaskDelay(10 / portTICK_PERIOD_MS);

  // define clock source
  ESP_ERROR_CHECK(mpu9250_set_clock_source(mpu9250_dev, MPU9250_CLOCK_PLL_XGYRO));
  vTaskDelay(10 / portTICK_PERIOD_MS);

  // define gyro range
  ESP_ERROR_CHECK(mpu9250_set_full_scale_gyro_range(mpu9250_dev, MPU9250_GYRO_FS_250));
  vTaskDelay(10 / portTICK_PERIOD_MS);

  // define accel range
  ESP_ERROR_CHECK(mpu9250_set_full_scale_accel_range(mpu9250_dev, MPU9250_ACCEL_FS_4));
  vTaskDelay(10 / portTICK_PERIOD_MS);

  // disable sleepEnabled
  ESP_ERROR_CHECK(mpu9250_set_sleep_enabled(mpu9250_dev, false));
  vTaskDelay(10 / portTICK_PERIOD_MS);

  ESP_LOGD(TAG, "END of MPU9250 initialization");

  mpu9250_print_settings(mpu9250_dev);

  return ESP_OK;
}

esp_err_t mpu9250_init_with_mag(i2c_dev_t *mpu9250_dev, i2c_dev_t *ak8963_dev, calibration_t *c)
{
    CHECK_ARG(mpu9250_dev);
    ESP_LOGI(TAG, "Initializating MPU9250");
    vTaskDelay(100 / portTICK_PERIOD_MS);

    if (initialised)
  {
    ESP_LOGE(TAG, "i2c_mpu9250_init has already been called");
    return ESP_ERR_INVALID_STATE;
  }
  initialised = true;
  cal = c;

  ESP_LOGD(TAG, "i2c_mpu9250_init");

  ESP_ERROR_CHECK(i2c_write_bit(mpu9250_dev, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_DEVICE_RESET_BIT, 1));
  vTaskDelay(10 / portTICK_PERIOD_MS);

  // define clock source
  ESP_ERROR_CHECK(mpu9250_set_clock_source(mpu9250_dev, MPU9250_CLOCK_PLL_XGYRO));
  vTaskDelay(10 / portTICK_PERIOD_MS);

  // define gyro range
  ESP_ERROR_CHECK(mpu9250_set_full_scale_gyro_range(mpu9250_dev, MPU9250_GYRO_FS_250));
  vTaskDelay(10 / portTICK_PERIOD_MS);

  // define accel range
  ESP_ERROR_CHECK(mpu9250_set_full_scale_accel_range(mpu9250_dev, MPU9250_ACCEL_FS_4));
  vTaskDelay(10 / portTICK_PERIOD_MS);

  // disable sleepEnabled
  ESP_ERROR_CHECK(mpu9250_set_sleep_enabled(mpu9250_dev, false));
  vTaskDelay(10 / portTICK_PERIOD_MS);

  ESP_LOGD(TAG, "END of MPU9250 initialization");

  ESP_ERROR_CHECK(mpu9250_enable_magnetometer(mpu9250_dev, ak8963_dev));

  mpu9250_print_settings_with_mag(mpu9250_dev, ak8963_dev);

  return ESP_OK;
}

esp_err_t mpu9250_set_clock_source(i2c_dev_t *mpu9250_dev, uint8_t adrs)
{
  return i2c_write_bits(mpu9250_dev, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CLKSEL_BIT, MPU9250_PWR1_CLKSEL_LENGTH, adrs);
}

esp_err_t mpu9250_set_full_scale_gyro_range(i2c_dev_t *mpu9250_dev, uint8_t adrs)
{
  gyro_inv_scale = mpu9250_get_gyro_inv_scale(adrs);
  return i2c_write_bits(mpu9250_dev, MPU9250_RA_GYRO_CONFIG, MPU9250_GCONFIG_FS_SEL_BIT, MPU9250_GCONFIG_FS_SEL_LENGTH, adrs);
}

esp_err_t mpu9250_set_full_scale_accel_range(i2c_dev_t *mpu9250_dev, uint8_t adrs)
{
  accel_inv_scale = mpu9250_get_accel_inv_scale(adrs);
  return i2c_write_bits(mpu9250_dev, MPU9250_RA_ACCEL_CONFIG_1, MPU9250_ACONFIG_FS_SEL_BIT, MPU9250_ACONFIG_FS_SEL_LENGTH, adrs);
}

esp_err_t mpu9250_set_sleep_enabled(i2c_dev_t *mpu9250_dev, bool state)
{
  return i2c_write_bit(mpu9250_dev, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_SLEEP_BIT, state ? 0x01 : 0x00);
}

esp_err_t mpu9250_set_bypass_enabled(i2c_dev_t *mpu9250_dev, bool state)
{
  return i2c_write_bit(mpu9250_dev, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_BYPASS_EN_BIT, state ? 1 : 0);
}

esp_err_t mpu9250_set_i2c_master_mode(i2c_dev_t *mpu9250_dev, bool state)
{
  return i2c_write_bit(mpu9250_dev, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_I2C_MST_EN_BIT, state ? 1 : 0);
}

esp_err_t mpu9250_get_clock_source(i2c_dev_t *mpu9250_dev, uint8_t *clock_source)
{
  uint8_t byte;
  esp_err_t ret = i2c_read_byte(mpu9250_dev, MPU9250_RA_PWR_MGMT_1, &byte);
  if (ret != ESP_OK)
  {
    return ret;
  }

  *clock_source = byte & 0x07;
  return ESP_OK;
}

float mpu9250_get_gyro_inv_scale(uint8_t scale_factor)
{
  switch (scale_factor)
  {
  case MPU9250_GYRO_FS_250:
    return 1.0 / MPU9250_GYRO_SCALE_FACTOR_0;
  case MPU9250_GYRO_FS_500:
    return 1.0 / MPU9250_GYRO_SCALE_FACTOR_1;
  case MPU9250_GYRO_FS_1000:
    return 1.0 / MPU9250_GYRO_SCALE_FACTOR_2;
  case MPU9250_GYRO_FS_2000:
    return 1.0 / MPU9250_GYRO_SCALE_FACTOR_3;
  default:
    ESP_LOGE(TAG, "get_gyro_inv_scale(): invalid value (%d)", scale_factor);
    return 1;
  }
}

float mpu9250_get_accel_inv_scale(uint8_t scale_factor)
{
  switch (scale_factor)
  {
  case MPU9250_ACCEL_FS_2:
    return 1.0 / MPU9250_ACCEL_SCALE_FACTOR_0;
  case MPU9250_ACCEL_FS_4:
    return 1.0 / MPU9250_ACCEL_SCALE_FACTOR_1;
  case MPU9250_ACCEL_FS_8:
    return 1.0 / MPU9250_ACCEL_SCALE_FACTOR_2;
  case MPU9250_ACCEL_FS_16:
    return 1.0 / MPU9250_ACCEL_SCALE_FACTOR_3;
  default:
    ESP_LOGE(TAG, "get_accel_inv_scale(): invalid value (%d)", scale_factor);
    return 1;
  }
}

esp_err_t mpu9250_get_sleep_enabled(i2c_dev_t *mpu9250_dev, bool *state)
{
  uint8_t bit;
  esp_err_t ret = i2c_read_bit(mpu9250_dev, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_SLEEP_BIT, &bit);
  if (ret != ESP_OK)
  {
    return ret;
  }
  *state = (bit == 0x01);

  return ESP_OK;
}

float scale_accel(float value, float offset, float scale_lo, float scale_hi)
{
  if (value < 0)
  {
    return -(value * accel_inv_scale - offset) / (scale_lo - offset);
  }
  else
  {
    return (value * accel_inv_scale - offset) / (scale_hi - offset);
  }
}

void align_accel(uint8_t bytes[6], vector_t *v)
{
  int16_t xi = BYTE_2_INT_BE(bytes, 0);
  int16_t yi = BYTE_2_INT_BE(bytes, 2);
  int16_t zi = BYTE_2_INT_BE(bytes, 4);

  v->x = scale_accel((float)xi, cal->accel_offset.x, cal->accel_scale_lo.x, cal->accel_scale_hi.x);
  v->y = scale_accel((float)yi, cal->accel_offset.y, cal->accel_scale_lo.y, cal->accel_scale_hi.y);
  v->z = scale_accel((float)zi, cal->accel_offset.z, cal->accel_scale_lo.z, cal->accel_scale_hi.z);
}

esp_err_t mpu9250_get_accel(i2c_dev_t *mpu9250_dev, vector_t *v)
{

  esp_err_t ret;
  uint8_t bytes[6];

  ret = i2c_read_bytes(mpu9250_dev, MPU9250_ACCEL_XOUT_H, bytes, 6);
  if (ret != ESP_OK)
  {
    return ret;
  }

  align_accel(bytes, v);

  return ESP_OK;
}

void align_gryo(uint8_t bytes[6], vector_t *v)
{
  int16_t xi = BYTE_2_INT_BE(bytes, 0);
  int16_t yi = BYTE_2_INT_BE(bytes, 2);
  int16_t zi = BYTE_2_INT_BE(bytes, 4);

  v->x = (float)xi * gyro_inv_scale + cal->gyro_bias_offset.x;
  v->y = (float)yi * gyro_inv_scale + cal->gyro_bias_offset.y;
  v->z = (float)zi * gyro_inv_scale + cal->gyro_bias_offset.z;
}

esp_err_t mpu9250_get_gyro(i2c_dev_t *mpu9250_dev, vector_t *v)
{
  esp_err_t ret;
  uint8_t bytes[6];
  ret = i2c_read_bytes(mpu9250_dev, MPU9250_GYRO_XOUT_H, bytes, 6);
  if (ret != ESP_OK)
  {
    return ret;
  }

  align_gryo(bytes, v);

  return ESP_OK;
}

esp_err_t mpu9250_get_accel_gyro(i2c_dev_t *mpu9250_dev, vector_t *va, vector_t *vg)
{
  esp_err_t ret;
  uint8_t bytes[14];
  ret = i2c_read_bytes(mpu9250_dev, MPU9250_ACCEL_XOUT_H, bytes, 14);
  if (ret != ESP_OK)
  {
    return ret;
  }

  // Accelerometer - bytes 0:5
  align_accel(bytes, va);

  // Skip Temperature - bytes 6:7

  // Gyroscope - bytes 9:13
  align_gryo(&bytes[8], vg);

  return ESP_OK;
}

esp_err_t mpu9250_get_accel_gyro_mag(i2c_dev_t *mpu9250_dev, i2c_dev_t *ak8963_dev, vector_t *va, vector_t *vg, vector_t *vm)
{
  esp_err_t ret;
  ret = mpu9250_get_accel_gyro(mpu9250_dev, va, vg);
  if (ret != ESP_OK)
  {
    return ret;
  }

  return ak8963_get_mag(ak8963_dev, vm);
}

esp_err_t mpu9250_get_mag(i2c_dev_t *ak8963_dev, vector_t *v)
{
  return ak8963_get_mag(ak8963_dev, v);
}

esp_err_t mpu9250_get_mag_raw(i2c_dev_t *ak8963_dev, uint8_t bytes[6])
{
  return ak8963_get_mag_raw(ak8963_dev, bytes);
}

esp_err_t mpu9250_get_device_id(i2c_dev_t *mpu9250_dev, uint8_t *val)
{
  return i2c_read_byte(mpu9250_dev, MPU9250_WHO_AM_I, val);
}

esp_err_t mpu9250_get_temperature_raw(i2c_dev_t *mpu9250_dev, uint16_t *val)
{
  uint8_t bytes[2];
  esp_err_t ret = i2c_read_bytes(mpu9250_dev, MPU9250_TEMP_OUT_H, bytes, 2);
  if (ret != ESP_OK)
  {
    return ret;
  }
  *val = BYTE_2_INT_BE(bytes, 0);
  return ESP_OK;
}

esp_err_t mpu9250_get_temperature_celsius(i2c_dev_t *mpu9250_dev, float *val)
{
  uint16_t raw_temp;
  esp_err_t ret = mpu9250_get_temperature_raw(mpu9250_dev, &raw_temp);
  if (ret != ESP_OK)
  {
    return ret;
  }
  *val = ((float)raw_temp) / 333.87 + 21.0;

  return ESP_OK;
}

static esp_err_t mpu9250_enable_magnetometer(i2c_dev_t *mpu9250_dev, i2c_dev_t *ak8963_dev)
{
  ESP_LOGI(TAG, "Enabling magnetometer");

  ESP_ERROR_CHECK(mpu9250_set_i2c_master_mode(mpu9250_dev, false));
  vTaskDelay(100 / portTICK_PERIOD_MS);

  ESP_ERROR_CHECK(mpu9250_set_bypass_enabled(mpu9250_dev, true));
  vTaskDelay(100 / portTICK_PERIOD_MS);

  bool is_enabled;
  ESP_ERROR_CHECK(mpu9250_get_bypass_enabled(mpu9250_dev, &is_enabled));
  if (is_enabled)
  {
    ak8963_init(ak8963_dev, cal);
    ESP_LOGI(TAG, "Magnetometer enabled");
    return ESP_OK;
  }
  else
  {
    ESP_LOGE(TAG, "Can't turn on RA_INT_PIN_CFG.");
    return ESP_ERR_INVALID_STATE;
  }
}

esp_err_t mpu9250_get_bypass_enabled(i2c_dev_t *mpu9250_dev, bool *state)
{
  uint8_t bit;
  esp_err_t ret = i2c_read_bit(mpu9250_dev, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_BYPASS_EN_BIT, &bit);
  if (ret != ESP_OK)
  {
    return ret;
  }
  *state = (bit == 0x01);

  return ESP_OK;
}

esp_err_t mpu9250_get_i2c_master_mode(i2c_dev_t *mpu9250_dev, bool *state)
{
  uint8_t bit;
  esp_err_t ret = i2c_read_bit(mpu9250_dev, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_I2C_MST_EN_BIT, &bit);
  if (ret != ESP_OK)
  {
    return ret;
  }
  *state = (bit == 0x01);

  return ESP_OK;
}

/**
 * @name get_gyro_power_settings
 */
esp_err_t mpu9250_get_gyro_power_settings(i2c_dev_t *mpu9250_dev, power_settings_e *ps)
{
  uint8_t byte;
  esp_err_t ret = i2c_read_byte(mpu9250_dev, MPU9250_RA_PWR_MGMT_2, &byte);
  if (ret != ESP_OK)
  {
    return ret;
  }

  byte = byte & 0x07;

  ps->x = (byte >> 2) & 1; // X
  ps->y = (byte >> 1) & 1; // Y
  ps->z = (byte >> 0) & 1; // Z

  return ESP_OK;
}

/**
 * @name get_accel_power_settings
 */
esp_err_t mpu9250_get_accel_power_settings(i2c_dev_t *mpu9250_dev, power_settings_e *ps)
{
  uint8_t byte;
  esp_err_t ret = i2c_read_byte(mpu9250_dev, MPU9250_RA_PWR_MGMT_2, &byte);
  if (ret != ESP_OK)
  {
    return ret;
  }

  byte = byte & 0x38;

  ps->x = (byte >> 5) & 1; // X
  ps->y = (byte >> 4) & 1; // Y
  ps->z = (byte >> 3) & 1; // Z

  return ESP_OK;
}

/**
 * @name get_full_scale_accel_range
 */
esp_err_t mpu9250_get_full_scale_accel_range(i2c_dev_t *mpu9250_dev, uint8_t *full_scale_accel_range)
{
  uint8_t byte;
  esp_err_t ret = i2c_read_byte(mpu9250_dev, MPU9250_RA_ACCEL_CONFIG_1, &byte);
  if (ret != ESP_OK)
  {
    return ret;
  }

  byte = byte & 0x18;
  byte = byte >> 3;

  *full_scale_accel_range = byte;

  return ESP_OK;
}

/**
 * @name get_full_scale_gyro_range
 */
esp_err_t mpu9250_get_full_scale_gyro_range(i2c_dev_t *mpu9250_dev, uint8_t *full_scale_gyro_range)
{
  uint8_t byte;
  esp_err_t ret = i2c_read_byte(mpu9250_dev, MPU9250_RA_GYRO_CONFIG, &byte);
  if (ret != ESP_OK)
  {
    return ret;
  }

  byte = byte & 0x18;
  byte = byte >> 3;

  *full_scale_gyro_range = byte;

  return ESP_OK;
}

#define YN(yn) (yn == 0 ? "Yes" : "No")

const char *CLK_RNG[] = {
    "0 (Internal 20MHz oscillator)",
    "1 (Auto selects the best available clock source)",
    "2 (Auto selects the best available clock source)",
    "3 (Auto selects the best available clock source)",
    "4 (Auto selects the best available clock source)",
    "5 (Auto selects the best available clock source)",
    "6 (Internal 20MHz oscillator)",
    "7 (Stops the clock and keeps timing generator in reset)"};

void mpu9250_print_chip_settings(i2c_dev_t *mpu9250_dev)
{

  uint8_t device_id;
  ESP_ERROR_CHECK(mpu9250_get_device_id(mpu9250_dev, &device_id));

  bool bypass_enabled;
  ESP_ERROR_CHECK(mpu9250_get_bypass_enabled(mpu9250_dev, &bypass_enabled));

  bool sleep_enabled;
  ESP_ERROR_CHECK(mpu9250_get_sleep_enabled(mpu9250_dev, &sleep_enabled));

  bool i2c_master_mode;
  ESP_ERROR_CHECK(mpu9250_get_i2c_master_mode(mpu9250_dev, &i2c_master_mode));

  uint8_t clock_source;
  ESP_ERROR_CHECK(mpu9250_get_clock_source(mpu9250_dev, &clock_source));

  power_settings_e accel_ps;
  ESP_ERROR_CHECK(mpu9250_get_accel_power_settings(mpu9250_dev, &accel_ps));

  power_settings_e gyro_ps;
  ESP_ERROR_CHECK(mpu9250_get_gyro_power_settings(mpu9250_dev, &gyro_ps));

  ESP_LOGI(TAG, "MPU9250:");
  ESP_LOGI(TAG, "--> Device address: 0x%02x", MPU9250_I2C_ADDR);
  ESP_LOGI(TAG, "--> Device ID: 0x%02x", device_id);
  ESP_LOGI(TAG, "--> initialised: %s", initialised ? "Yes" : "No");
  ESP_LOGI(TAG, "--> BYPASS enabled: %s", bypass_enabled ? "Yes" : "No");
  ESP_LOGI(TAG, "--> SleepEnabled Mode: %s", sleep_enabled ? "On" : "Off");
  ESP_LOGI(TAG, "--> i2c Master Mode: %s", i2c_master_mode ? "Enabled" : "Disabled");
  ESP_LOGI(TAG, "--> Power Management (0x6B, 0x6C):");
  ESP_LOGI(TAG, "  --> Clock Source: %d %s", clock_source, CLK_RNG[clock_source]);
  ESP_LOGI(TAG, "  --> Accel enabled (x, y, z): (%s, %s, %s)",
           YN(accel_ps.x),
           YN(accel_ps.y),
           YN(accel_ps.z));
  ESP_LOGI(TAG, "  --> Gyro enabled (x, y, z): (%s, %s, %s)",
           YN(gyro_ps.x),
           YN(gyro_ps.y),
           YN(gyro_ps.z));
}

const char *FS_RANGE[] = {"±2g (0)", "±4g (1)", "±8g (2)", "±16g (3)"};

void mpu9250_print_accel_settings(i2c_dev_t *mpu9250_dev)
{
  uint8_t full_scale_accel_range;
  ESP_ERROR_CHECK(mpu9250_get_full_scale_accel_range(mpu9250_dev, &full_scale_accel_range));

  ESP_LOGI(TAG, "Accelerometer:");
  ESP_LOGI(TAG, "--> Full Scale Range (0x1C): %s", FS_RANGE[full_scale_accel_range]);
  ESP_LOGI(TAG, "--> Scalar: 1/%f", 1.0 / accel_inv_scale);
  ESP_LOGI(TAG, "--> Calibration:");
  ESP_LOGI(TAG, "  --> Offset: ");
  ESP_LOGI(TAG, "    --> x: %f", cal->accel_offset.x);
  ESP_LOGI(TAG, "    --> y: %f", cal->accel_offset.y);
  ESP_LOGI(TAG, "    --> z: %f", cal->accel_offset.z);
  ESP_LOGI(TAG, "  --> Scale: ");
  ESP_LOGI(TAG, "    --> x: (%f, %f)", cal->accel_scale_lo.x, cal->accel_scale_hi.x);
  ESP_LOGI(TAG, "    --> y: (%f, %f)", cal->accel_scale_lo.y, cal->accel_scale_hi.y);
  ESP_LOGI(TAG, "    --> z: (%f, %f)", cal->accel_scale_lo.z, cal->accel_scale_hi.z);
};

void mpu9250_print_gyro_settings(i2c_dev_t *mpu9250_dev)
{
  const char *FS_RANGE[] = {
      "+250 dps (0)",
      "+500 dps (1)",
      "+1000 dps (2)",
      "+2000 dps (3)"};

  uint8_t full_scale_gyro_range;
  ESP_ERROR_CHECK(mpu9250_get_full_scale_gyro_range(mpu9250_dev, &full_scale_gyro_range));

  ESP_LOGI(TAG, "Gyroscope:");
  ESP_LOGI(TAG, "--> Full Scale Range (0x1B): %s", FS_RANGE[full_scale_gyro_range]);
  ESP_LOGI(TAG, "--> Scalar: 1/%f", 1.0 / gyro_inv_scale);
  ESP_LOGI(TAG, "--> Bias Offset:");
  ESP_LOGI(TAG, "  --> x: %f", cal->gyro_bias_offset.x);
  ESP_LOGI(TAG, "  --> y: %f", cal->gyro_bias_offset.y);
  ESP_LOGI(TAG, "  --> z: %f", cal->gyro_bias_offset.z);
};

void mpu9250_print_settings(i2c_dev_t *mpu9250_dev)
{
  mpu9250_print_chip_settings(mpu9250_dev);
  mpu9250_print_accel_settings(mpu9250_dev);
  mpu9250_print_gyro_settings(mpu9250_dev);
}

void mpu9250_print_settings_with_mag(i2c_dev_t *mpu9250_dev, i2c_dev_t *ak8963_dev)
{
  mpu9250_print_chip_settings(mpu9250_dev);
  mpu9250_print_accel_settings(mpu9250_dev);
  mpu9250_print_gyro_settings(mpu9250_dev);
  ak8963_print_settings(ak8963_dev);
}
