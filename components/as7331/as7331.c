/*
 * Copyright (c) 2021 Bhavya Desai <bhavya.p.desai2002@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file as7331.c
 *
 * ESP-IDF driver for Light/UV Index, UV radiation sensors AS7331
 *
 * Copyright (c) 2021 Bhavya Desai <bhavya.p.desai2002@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "as7331.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdint.h>
#include <esp_idf_lib_helpers.h>

#define I2C_FREQ_HZ 400000 // 400kHz

/**
 * Control/Configuration register bank
 */
#define REG_ADDR_OSR (0x00)     // Operational State Register
#define REG_ADDR_AGEN (0x02)    // API Generation Register
#define REG_ADDR_CREG1 (0x06)   // Configuration Register 1
#define REG_ADDR_CREG2 (0x07)   // Configuration Register 2
#define REG_ADDR_CREG3 (0x08)   // Configuration Register 3
#define REG_ADDR_BREAK (0x09)   // Break Register
#define REG_ADDR_EDGES (0x0A)   // Edges Register (for SYND mode)
#define REG_ADDR_OPTREG (0x0B)  // Option Register

/**
 * Output register bank (Measurement State)
 */
#define REG_ADDR_STATUS (0x00)      // Status register 
#define REG_ADDR_TEMP (0x01)        // Temperature measurement result
#define REG_ADDR_MRES1 (0x02)       // Channel A measurement result
#define REG_ADDR_MRES2 (0x03)       // Channel B measurement result
#define REG_ADDR_MRES3 (0x04)       // Channel C measurement result
#define REG_ADDR_OUTCONVL (0x05)    // Result of time conversion (lsb) 
#define REG_ADDR_OUTCONVH (0x06)    // Result of time conversion (msb)

/**
 * Device Modes and Allowed values
 */
#define DEVICE_STATE_CONFIGURATION (0x2)
#define DEVICE_STATE_MEASUREMENT   (0x3)

/**
 * Minimum and Maximum value of Integration time
 */
#define INTEGRATION_TIME_MIN_VAL (INTEGRATION_TIME_1MS)
#define INTEGRATION_TIME_MAX_VAL (INTEGRATION_TIME_16384MS) 

/**
 * Minimum and maximum gain values
 */
#define GAIN_MIN_VAL (0x00)
#define GAIN_MAX_VAL (0x0b)
#define GAIN_BIT_SHIFT (4)

/**
 * Minimum and Maximum Internal Frequency value
 */
#define CCLK_MIN_VAL CCLK_FREQ_1024KHZ
#define CCKL_MAX_VAL CCLK_FREQ_8192KHZ

/**
 * Fullscale range values for UV A, B and C channels
 */
#define FSRA (348160)
#define FSRB (387072)
#define FSRC (169984)

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define SLEEP_MS(x) do { vTaskDelay(pdMS_TO_TICKS(x)); } while (0)
#define CHECK_LOGE(x, msg, ...) do { \
        esp_err_t __; \
        if ((__ = x) != ESP_OK) { \
            ESP_LOGE(TAG, msg, ## __VA_ARGS__); \
            return __; \
        } \
    } while (0)

static const char *TAG = "as7331";

////////// I2C Read and Write Functions //////////

// Read a single byte from the specified AS7331 register over I2C
static esp_err_t read_register(as7331_t *dev, uint8_t reg, uint8_t *value) {
    esp_err_t err;
    err = i2c_dev_read_reg(&dev->i2c_dev, reg, value, 1);
    ESP_LOGD(TAG, "Read register: 0x%x; Data: 0x%x.", reg, *value);
    return err;
}

// Write a single byte to the specified AS7331 register over I2C
static esp_err_t write_register(as7331_t *dev, uint8_t reg, uint8_t value) {
    ESP_LOGD(TAG, "Writing register: 0x%x; Data: 0x%x.", reg, value);
    return i2c_dev_write_reg(&dev->i2c_dev, reg, &value, 1);
}

// Read a 16-bit little-endian value starting from the given register address
static esp_err_t read_register16(as7331_t *dev, uint8_t low_register, uint16_t *value) {
    uint8_t buf[2];
    CHECK(i2c_dev_read_reg(&dev->i2c_dev, low_register, buf, 2));
    *value = (uint16_t)buf[1] << 8 | buf[0];
    return ESP_OK;
}

// Update specific bits in a register using a read-modify-write sequence
static esp_err_t write_register_bits(as7331_t *dev, uint8_t reg, uint8_t mask, uint8_t bits, uint8_t start_pos) {
    uint8_t value;
    CHECK(read_register(dev, reg, &value));
    value &= ~mask;
    value |= (bits << start_pos) & mask;
    CHECK(write_register(dev, reg, value));
    return ESP_OK;
}

////////// Registers read/write functions //////////

// Set the device operating state (configuration or measurement mode)
static esp_err_t as7331_set_device_state_internal(as7331_t *dev, uint8_t value) {
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_OSR, OSR_MASK_DOS, value, 0));
    return ESP_OK;
}

// Read the current device operating state from the OSR register
static esp_err_t as7331_get_device_state(as7331_t *dev, uint8_t *value) {
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, REG_ADDR_OSR, value));
    *value = *value & OSR_MASK_DOS;
    return ESP_OK;
}

// Read the contents of Configuration Register 1 (CREG1)
static esp_err_t as7331_get_creg1(as7331_t *dev, uint8_t *value) {
    CHECK(read_register(dev, REG_ADDR_CREG1, value));
    return ESP_OK;
}

// Write a value to Configuration Register 1 (CREG1)
static esp_err_t as7331_set_creg1(as7331_t *dev, uint8_t value) {
    CHECK(write_register(dev, REG_ADDR_CREG1, value));
    return ESP_OK;
}

// Read the contents of Configuration Register 2 (CREG2)
static esp_err_t as7331_get_creg2(as7331_t *dev, uint8_t *value) {
    CHECK(read_register(dev, REG_ADDR_CREG2, value));
    return ESP_OK;
}

// Write a value to Configuration Register 2 (CREG2)
static esp_err_t as7331_set_creg2(as7331_t *dev, uint8_t value) {
    CHECK(write_register(dev, REG_ADDR_CREG2, value));
    return ESP_OK;
}

// Read the contents of Configuration Register 3 (CREG3)
static esp_err_t as7331_get_creg3(as7331_t *dev, uint8_t *value) {
    CHECK(read_register(dev, REG_ADDR_CREG3, value));
    return ESP_OK;
}

// Write a value to Configuration Register 3 (CREG3)
static esp_err_t as7331_set_creg3(as7331_t *dev, uint8_t value) {
    CHECK(write_register(dev, REG_ADDR_CREG3, value));
    return ESP_OK;
}

// Read the raw temperature measurement value
static esp_err_t as7331_get_temperature(as7331_t *dev, uint16_t *value) {
    CHECK(read_register16(dev, REG_ADDR_TEMP, value));
    return ESP_OK;
}

// Read the raw measurement result for the UVA channel
static esp_err_t as7331_get_mres1(as7331_t *dev, uint16_t *value) {
    CHECK(read_register16(dev, REG_ADDR_MRES1, value));
    return ESP_OK;
}

// Read the raw measurement result for the UVB channel
static esp_err_t as7331_get_mres2(as7331_t *dev, uint16_t *value) {
    CHECK(read_register16(dev, REG_ADDR_MRES2, value));
    return ESP_OK;
}

// Read the raw measurement result for the UVC channel
static esp_err_t as7331_get_mres3(as7331_t *dev, uint16_t *value) {
    CHECK(read_register16(dev, REG_ADDR_MRES3, value));
    return ESP_OK;
}

// Enable or disable measurement of the integration time
// Enables(1) or disables(0)
static esp_err_t as7331_set_time_measurement_enabled(as7331_t *dev, uint8_t value) {
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_CREG2, CREG2_MASK_EN_TM, value, 2));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    return ESP_OK;
}

// Read the integration time measurement enable state
static esp_err_t as7331_get_time_measurement_enabled(as7331_t *dev, uint8_t *value) {
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    CHECK(as7331_get_creg2(dev, value));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    *value = *value & CREG2_MASK_EN_TM;
    return ESP_OK;
}

// Write a raw value to the Operational State Register (OSR)
static esp_err_t as7331_set_osr(as7331_t *dev, uint8_t value) {
    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, REG_ADDR_OSR, value));
    return ESP_OK;
}

// Read the current value of the Operational State Register (OSR)
static esp_err_t as7331_get_osr(as7331_t *dev, uint8_t *value) {
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, REG_ADDR_OSR, value));
    return ESP_OK;
}

// Read both OSR and STATUS registers in a single 16-bit access
static esp_err_t as7331_get_osr_and_status(as7331_t *dev, uint16_t *value) {
    I2C_DEV_CHECK(&dev->i2c_dev, read_register16(dev, REG_ADDR_STATUS, value));
    return ESP_OK;
}

// Read the contents of the STATUS register
static esp_err_t as7331_get_status(as7331_t *dev, uint16_t *value) {
    I2C_DEV_CHECK(&dev->i2c_dev, read_register16(dev, REG_ADDR_STATUS, value));
    return ESP_OK;
}

// Decode the STATUS register bits into the device status structure
static esp_err_t as7331_get_status_as_dict(as7331_t *dev, uint16_t *status_byte) {
    CHECK(as7331_get_status(dev, status_byte));
    dev->status.powerstate = STATUS_MASK_POWERSTATE & *status_byte;
    dev->status.standbystate = STATUS_MASK_STANDBYSTATE & *status_byte;
    dev->status.notready = STATUS_MASK_NOTREADY & *status_byte;
    dev->status.ndata = STATUS_MASK_NDATA & *status_byte;
    dev->status.ldata = STATUS_MASK_LDATA & *status_byte;
    dev->status.adcof = STATUS_MASK_ADCOF & *status_byte;
    dev->status.mresof = STATUS_MASK_MRESOF & *status_byte;
    dev->status.outconvof = STATUS_MASK_OUTCONVOF & *status_byte;
    return ESP_OK;
}

// Check whether the device is currently not ready for a new measurement
static esp_err_t as7331_not_ready(as7331_t *dev, bool *not_ready) {
    uint16_t value;
    CHECK(as7331_get_status(dev, &value));
    *not_ready = (STATUS_MASK_NOTREADY & value) != 0;
    return ESP_OK;
}

////////// Calculations Functions //////////

// Convert the gain register setting to its corresponding numeric gain factor
static uint16_t as7331_calculate_gain_value(uint16_t gain) {
    return (1 << (11 - gain));
}

// Convert the internal clock configuration to its frequency value in kHz
static float as7331_calculate_cclk_value(uint16_t cclk) {
    return (1024.0*(1 << cclk));
}

// Convert the integration time setting to its duration in milliseconds
static uint16_t as7331_calculate_integration_time_value(uint16_t integration_time) {
    return ((1 << integration_time));
}

// Convert the raw temperature measurement to degrees Celsius
static float as7331_calculate_temperature_celsius_value(uint16_t temp) {
    return (0.05*temp - 66.9);
}

// Compute the common conversion factor for raw UV measurements
static float as7331_get_conversation_factor(as7331_t *dev) {
    uint16_t gain = as7331_calculate_gain_value(dev->settings.gain);
    float cclk = as7331_calculate_cclk_value(dev->settings.cclk);
    uint16_t time = as7331_calculate_integration_time_value(dev->settings.integration_time);
    return 1.0/(gain * time * cclk);
}

// Compute the numeric scale factor applied by the measurement result divider
static uint16_t as7331_calculate_divider_factor(as7331_t *dev) {
    if(dev->settings.divider_enable) {
        return 1 << (1 + dev->settings.divider);
    } 
    else {
        return 1;
    }
}

// Compute the required delay time for a measurement based on integration time
static float as7331_measurement_sleep_dt(as7331_t *dev) {
    return 0.001* as7331_calculate_integration_time_value(dev->settings.integration_time);
}

////////// Radiation measurement and measurement starting registers //////////

// Trigger a measurement by setting the start bit in the OSR register
static esp_err_t as7331_start_measurement(as7331_t *dev) {
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_OSR, OSR_MASK_SS, 1, 7));
    return ESP_OK;
}

// Start a single measurement and wait until the sensor reports ready
static esp_err_t as7331_start_single_measurement(as7331_t *dev) {
    CHECK(as7331_start_measurement(dev));
    bool not_ready;
    uint32_t measurement_sleep = as7331_measurement_sleep_dt(dev);
    vTaskDelay(pdMS_TO_TICKS(measurement_sleep));
    while (as7331_not_ready(dev, &not_ready)) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_OK;
}

// Perform a single measurement and return the raw UVA channel value
static float as7331_get_uva_raw(as7331_t *dev) {
    as7331_start_single_measurement(dev);
    uint16_t div_factor = 0;
    uint16_t uv = 0;
    div_factor = as7331_calculate_divider_factor(dev);
    as7331_get_mres1(dev, &uv);
    float uva_raw = uv * div_factor;
    return uva_raw;
}

// Perform a single measurement and return the raw UVB channel value
static float as7331_get_uvb_raw(as7331_t *dev) {
    as7331_start_single_measurement(dev);
    uint16_t div_factor = 0;
    uint16_t uv = 0;
    div_factor = as7331_calculate_divider_factor(dev);
    as7331_get_mres2(dev, &uv);
    float uvb_raw = uv * div_factor;
    return uvb_raw;
}

// Perform a single measurement and return the raw UVC channel value
static float as7331_get_uvc_raw(as7331_t *dev) {
    as7331_start_single_measurement(dev);
    uint16_t div_factor = 0;
    uint16_t uv = 0;
    div_factor = as7331_calculate_divider_factor(dev);
    as7331_get_mres3(dev, &uv);
    float uvc_raw = uv * div_factor;
    return uvc_raw;
}

// Perform a measurement and read all raw UV channels and temperature values
static esp_err_t as7331_get_uv_raw_internal(as7331_t *dev, as7331_raw_values_t *out) {
    CHECK(as7331_start_measurement(dev));

    bool not_ready = true;
    uint32_t measurement_sleep = as7331_measurement_sleep_dt(dev);
    vTaskDelay(pdMS_TO_TICKS(measurement_sleep));
    while (not_ready) {
        CHECK(as7331_not_ready(dev, &not_ready));
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    uint32_t div_factor = 0;
    uint16_t uv = 0;
    div_factor = as7331_calculate_divider_factor(dev);
    CHECK(as7331_get_mres1(dev, &uv));
    out->uva_raw = (float)uv * div_factor;
    CHECK(as7331_get_mres2(dev, &uv));
    out->uvb_raw = (float)uv * div_factor;
    CHECK(as7331_get_mres3(dev, &uv));
    out->uvc_raw = (float)uv * div_factor;

    CHECK(as7331_get_temperature(dev, &out->temp_raw));
    return ESP_OK;
}

////////// No Lock Functions for public functions//////////

// Perform a software reset by setting the OSR reset bit (mutex must be held)
static esp_err_t as7331_software_reset_nolock(as7331_t *dev) {
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_OSR, OSR_MASK_SW_RES, 1, 3));
    return ESP_OK;
}

// Read the raw chip ID field from the AGEN register (mutex must be held)
static esp_err_t as7331_get_chip_id_nolock(as7331_t * dev, uint8_t *value) {
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, REG_ADDR_AGEN, value));
    return ESP_OK;
}

// Set the measurement mode in CREG3, handling config/measurement state transitions (mutex must be held)
static esp_err_t as7331_set_measurement_mode_nolock(as7331_t *dev, uint8_t value) {
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_CREG3, CREG3_MASK_MMODE, value, 6));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    dev->settings.measurement_mode = value;
    return ESP_OK;
}

// Read the current measurement mode from CREG3 (mutex must be held)
static esp_err_t as7331_get_measurement_mode_nolock(as7331_t *dev, uint8_t *value) {
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    CHECK(as7331_get_creg3(dev, value));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    *value = *value & CREG3_MASK_MMODE;
    return ESP_OK;
}

// Configure the integration time in CREG1 and update cached settings (mutex must be held)
static esp_err_t as7331_set_integration_time_nolock(as7331_t *dev, uint8_t value) {
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_CREG1, CREG1_MASK_INTEGRATION_TIME, value, 0));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    dev->settings.integration_time = value;
    return ESP_OK;
}

// Read the integration time field from CREG1 (mutex must be held)
static esp_err_t as7331_get_integration_time_nolock(as7331_t *dev, uint8_t *value) {
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    CHECK(as7331_get_creg1(dev, value));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    *value = *value & CREG1_MASK_INTEGRATION_TIME;
    return ESP_OK;
}

// Configure the analog gain in CREG1 and update cached settings (mutex must be held)
static esp_err_t as7331_set_gain_nolock(as7331_t *dev, uint8_t value) {
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_CREG1, CREG1_MASK_GAIN, value, GAIN_BIT_SHIFT));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    dev->settings.gain = value;
    return ESP_OK;
}

// Read the current analog gain setting from CREG1 (mutex must be held)
static esp_err_t as7331_get_gain_nolock(as7331_t *dev, uint8_t *value) {
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    CHECK(as7331_get_creg1(dev, value));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    *value = (*value & CREG1_MASK_GAIN) >> GAIN_BIT_SHIFT;
    return ESP_OK;
}

// Enable or disable standby mode via CREG3 and update cached settings (mutex must be held)
static esp_err_t as7331_set_standby_state_nolock(as7331_t *dev, bool value) {
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_CREG3, CREG3_MASK_SB, value, 4));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    dev->settings.standby_state = value;
    return ESP_OK;
}

// Read the standby mode bit from CREG3 (mutex must be held)
static esp_err_t as7331_get_standby_state_nolock(as7331_t *dev, uint8_t *value) {
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    CHECK(as7331_get_creg3(dev, value));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    *value = *value & CREG3_MASK_SB;
    return ESP_OK;
}

// Enable or disable power-down mode via OSR and update cached settings (mutex must be held)
static esp_err_t as7331_set_power_mode_nolock(as7331_t *dev, bool value) {
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_OSR, OSR_MASK_PD, (uint8_t)value, 5));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    dev->settings.power_mode = value;
    return ESP_OK;
}

// Read the power-down mode bit from OSR (mutex must be held)
static esp_err_t as7331_get_power_mode_nolock(as7331_t *dev, uint8_t *value) {
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    CHECK(as7331_get_osr(dev, value));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    *value = *value & OSR_MASK_PD;
    return ESP_OK;
}

// Enable or disable the measurement result divider via CREG2 (mutex must be held)
static esp_err_t as7331_set_divider_enabled_nolock(as7331_t *dev, bool value) {
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_CREG2, CREG2_MASK_EN_DIV, value, 3));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    dev->settings.divider_enable = value;
    return ESP_OK;
}

// Read the divider enable bit from CREG2 (mutex must be held)
static esp_err_t as7331_get_divider_enabled_nolock(as7331_t *dev, uint8_t *value) {
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    CHECK(as7331_get_creg2(dev, value));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    *value = *value & CREG2_MASK_EN_DIV;
    return ESP_OK;
}


// Configure the measurement result divider value in CREG2 and update cached settings (mutex must be held)
static esp_err_t as7331_set_divider_nolock(as7331_t *dev, uint8_t value) {
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_CREG2, CREG2_MASK_DIV, value, 0));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    dev->settings.divider = value;
    return ESP_OK;
}

// Read the measurement result divider value from CREG2 (mutex must be held)
static esp_err_t as7331_get_divider_nolock(as7331_t *dev, uint8_t *value) {
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    CHECK(as7331_get_creg2(dev, value));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    *value = *value & CREG2_MASK_DIV;
    return ESP_OK;
}

// Configure the internal clock frequency in CREG3 and update cached settings (mutex must be held)
static esp_err_t as7331_set_cclk_nolock(as7331_t *dev, uint8_t value) {
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_CREG3, CREG3_MASK_CCLK, value, 0));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    dev->settings.cclk = value;
    return ESP_OK;
}

// Read the internal clock frequency field from CREG3 (mutex must be held)
static esp_err_t as7331_get_cclk_nolock(as7331_t *dev, uint8_t *value) {
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    CHECK(as7331_get_creg3(dev, value));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    *value = *value & CREG3_MASK_CCLK;
    return ESP_OK;
}

// Apply the driver’s default configuration sequence without taking the mutex
esp_err_t as7331_set_default_config_nolock(as7331_t *dev) {
    CHECK(as7331_set_measurement_mode_nolock(dev, MEASUREMENT_MODE_COMMAND));
    CHECK(as7331_set_integration_time_nolock(dev, INTEGRATION_TIME_256MS));
    CHECK(as7331_set_gain_nolock(dev, GAIN_16X));
    CHECK(as7331_set_standby_state_nolock(dev, false));
    CHECK(as7331_set_power_mode_nolock(dev, false));
    CHECK(as7331_set_divider_enabled_nolock(dev, false));
    CHECK(as7331_set_cclk_nolock(dev, CCLK_FREQ_1024KHZ));
    return ESP_OK;
}

///////////////////// Public Functions ////////////////////

esp_err_t as7331_init_desc(as7331_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr != AS7331_I2C_ADDR_0 &&  addr != AS7331_I2C_ADDR_1 && addr != AS7331_I2C_ADDR_2 && addr != AS7331_I2C_ADDR_3)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t as7331_free_desc(as7331_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t as7331_software_reset(as7331_t *dev) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_OSR, OSR_MASK_SW_RES, 1, 3));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

esp_err_t as7331_get_chip_id(as7331_t * dev, uint8_t *value) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, REG_ADDR_AGEN, value));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

esp_err_t as7331_set_measurement_mode(as7331_t *dev, uint8_t value) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_CREG3, CREG3_MASK_MMODE, value, 6));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    dev->settings.measurement_mode = value;
    return ESP_OK;
}

esp_err_t as7331_get_measurement_mode(as7331_t *dev, uint8_t *value) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    CHECK(as7331_get_creg3(dev, value));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *value = *value & CREG3_MASK_MMODE;
    return ESP_OK;
}

esp_err_t as7331_set_integration_time(as7331_t *dev, uint8_t value) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_CREG1, CREG1_MASK_INTEGRATION_TIME, value, 0));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    dev->settings.integration_time = value;
    return ESP_OK;
}

esp_err_t as7331_get_integration_time(as7331_t *dev, uint8_t *value) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    CHECK(as7331_get_creg1(dev, value));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *value = *value & CREG1_MASK_INTEGRATION_TIME;
    return ESP_OK;
}

esp_err_t as7331_set_gain(as7331_t *dev, uint8_t value) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_CREG1, CREG1_MASK_GAIN, value, GAIN_BIT_SHIFT));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    dev->settings.gain = value;
    return ESP_OK;
}

esp_err_t as7331_get_gain(as7331_t *dev, uint8_t *value) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    CHECK(as7331_get_creg1(dev, value));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *value = (*value & CREG1_MASK_GAIN) >> GAIN_BIT_SHIFT;
    return ESP_OK;
}

esp_err_t as7331_set_standby_state(as7331_t *dev, bool value) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_CREG3, CREG3_MASK_SB, value, 4));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    dev->settings.standby_state = value;
    return ESP_OK;
}

esp_err_t as7331_get_standby_state(as7331_t *dev, uint8_t *value) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    CHECK(as7331_get_creg3(dev, value));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *value = *value & CREG3_MASK_SB;
    return ESP_OK;
}

esp_err_t as7331_set_power_mode(as7331_t *dev, bool value) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_OSR, OSR_MASK_PD, (uint8_t)value, 5));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    dev->settings.power_mode = value;
    return ESP_OK;
}

esp_err_t as7331_get_power_mode(as7331_t *dev, uint8_t *value) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    CHECK(as7331_get_osr(dev, value));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *value = *value & OSR_MASK_PD;
    return ESP_OK;
}

esp_err_t as7331_set_divider_enabled(as7331_t *dev, bool value) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_CREG2, CREG2_MASK_EN_DIV, value, 3));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    dev->settings.divider_enable = value;
    return ESP_OK;
}

esp_err_t as7331_get_divider_enabled(as7331_t *dev, uint8_t *value) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    CHECK(as7331_get_creg2(dev, value));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *value = *value & CREG2_MASK_EN_DIV;
    return ESP_OK;
}

esp_err_t as7331_set_divider(as7331_t *dev, uint8_t value) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_CREG2, CREG2_MASK_DIV, value, 0));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    dev->settings.divider = value;
    return ESP_OK;
}

esp_err_t as7331_get_divider(as7331_t *dev, uint8_t *value) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    CHECK(as7331_get_creg2(dev, value));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *value = *value & CREG2_MASK_DIV;
    return ESP_OK;
}

esp_err_t as7331_set_cclk(as7331_t *dev, uint8_t value) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_bits(dev, REG_ADDR_CREG3, CREG3_MASK_CCLK, value, 0));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    dev->settings.cclk = value;
    return ESP_OK;
}

esp_err_t as7331_get_cclk(as7331_t *dev, uint8_t *value) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_CONFIGURATION));
    CHECK(as7331_get_creg3(dev, value));
    CHECK(as7331_set_device_state_internal(dev, DEVICE_STATE_MEASUREMENT));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *value = *value & CREG3_MASK_CCLK;
    return ESP_OK;
}

esp_err_t as7331_set_default_config(as7331_t *dev) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    CHECK(as7331_set_measurement_mode_nolock(dev, MEASUREMENT_MODE_COMMAND));
    CHECK(as7331_set_integration_time_nolock(dev, INTEGRATION_TIME_256MS));
    CHECK(as7331_set_gain_nolock(dev, GAIN_16X));
    CHECK(as7331_set_standby_state_nolock(dev, false));
    CHECK(as7331_set_power_mode_nolock(dev, false));
    CHECK(as7331_set_divider_enabled_nolock(dev, false));
    CHECK(as7331_set_cclk_nolock(dev, CCLK_FREQ_1024KHZ));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t as7331_init_sensor(as7331_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    dev->settings.divider_enable = false;
    dev->settings.power_mode = false;
    dev->settings.standby_state = false;
    dev->settings.divider = 1;
    dev->settings.cclk = CCLK_FREQ_1024KHZ;
    dev->settings.integration_time = INTEGRATION_TIME_256MS;
    dev->settings.gain = GAIN_16X;

    // reset the sensor
    CHECK(as7331_software_reset_nolock(dev));
    vTaskDelay(pdMS_TO_TICKS(AS7331_RESET_PERIOD));

    CHECK(as7331_get_chip_id_nolock(dev, &dev->chip_id));
    dev->chip_id = dev->chip_id >> AGEN_SHIFT_DEVID;

    CHECK(as7331_set_default_config_nolock(dev));

    CHECK(as7331_set_gain_nolock(dev, GAIN_512X));
    CHECK(as7331_set_integration_time_nolock(dev, INTEGRATION_TIME_128MS));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t as7331_get_uv_values(as7331_t *dev, as7331_values_float_t *out) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    float common_factor = as7331_get_conversation_factor(dev);
    float conv_factor_a = FSRA*common_factor;
    float conv_factor_b = FSRB*common_factor;
    float conv_factor_c = FSRC*common_factor;

    as7331_raw_values_t raw;
    CHECK(as7331_get_uv_raw_internal(dev, &raw));
    out->uv_a = (raw.uva_raw * conv_factor_a) / 100;
    out->uv_b = (raw.uvb_raw * conv_factor_b) / 100;
    out->uv_c = (raw.uvc_raw * conv_factor_c) / 100;
    out->temperature = as7331_calculate_temperature_celsius_value(raw.temp_raw);

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    float weight_a = 0.0134;
    float weight_b = 0.77;
    float weight_c = 1;
    float divider = 0.025;

    out->uv_index = ((out->uv_a * weight_a) + (out->uv_b * weight_b) + (out->uv_c * weight_c)) / divider;

    return ESP_OK;
}
