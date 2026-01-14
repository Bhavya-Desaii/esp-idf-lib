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
 * @file as7331.h
 * @defgroup as7331 as7331
 * @{
 *
 * ESP-IDF driver for Light/UV Index, UV radiation sensors AS7331
 *
 * Copyright (c) 2021 Bhavya Desai <bhavya.p.desai2002@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __AS7331_H__
#define __AS7331_H__

#include <i2cdev.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 7-bit address defined as [1, 1, 1, 0, 1, A1, A0] where A1/A0 are the
 * physical address pins tied high or low
 */
#define AS7331_I2C_ADDR_0 (0x74)  // AS7331 I2C Address A1 = 0, A0 = 0
#define AS7331_I2C_ADDR_1 (0x75)  // AS7331 I2C Address A1 = 0, A0 = 1
#define AS7331_I2C_ADDR_2 (0x76)  // AS7331 I2C Address A1 = 1, A0 = 0
#define AS7331_I2C_ADDR_3 (0x77)  // AS7331 I2C Address A1 = 1, A0 = 1

/**
 * Reset time delay
 */
#define AS7331_RESET_PERIOD 2

/**
 * OSR REG Masks (Rear/Write)
 */
typedef enum {
    OSR_MASK_DOS      = 0x07,       // Device Operating State 
    OSR_MASK_SW_RES   = 0x08,       // Software Reset 
    OSR_MASK_PD       = 0x40,       // Power Down State Switch 
    OSR_MASK_SS       = 0x80        // Start/Stop measurement
} as7331_osr_reg_mask_t;

/**
 * Control Register 1 Mask (Read/Write)
 */
typedef enum {
    CREG1_MASK_INTEGRATION_TIME = 0x0f,     // Integration time
    CREG1_MASK_GAIN             = 0xf0      // Irradiance reponsivity
} as7331_creg1_mask_t;

/**
 * Control Register 2 mask
 */
typedef enum {
    CREG2_MASK_EN_TM  = 0x40,   // Disable/enable measurement of integration time
    CREG2_MASK_EN_DIV = 0x08,   // Disable/enable divider of measurement results
    CREG2_MASK_DIV    = 0x07    // Digital divider of measurement results
} as7331_creg2_mask_t;

/**
 * Control Register 3 mask
 */
typedef enum {
    CREG3_MASK_MMODE = 0xc0,    // Measurement mode
    CREG3_MASK_SB    = 0x10,    // Standby disable/enable
    CREG3_MASK_RDYOD = 0x08,    // Pin Ready push-pull or open-drain
    CREG3_MASK_CCLK  = 0x03     // Internal clock frequency
} as7331_creg3_mask_t;

/**
 * Status Register mask
 */
typedef enum {
    STATUS_MASK_POWERSTATE   = 0x01,    // Power state  
    STATUS_MASK_STANDBYSTATE = 0x02,    // Standby state 
    STATUS_MASK_NOTREADY     = 0x04,    // Not ready state
    STATUS_MASK_NDATA        = 0x08,    // New measurements transferred
    STATUS_MASK_LDATA        = 0x10,    // Measurements overwritten
    STATUS_MASK_ADCOF        = 0x20,    // Overflow of internal conversion channel
    STATUS_MASK_MRESOF       = 0x40,    // Overflow of measurement result
    STATUS_MASK_OUTCONVOF    = 0x80     // Overflow of time reference
} as7331_status_reg_mask_t;

/**
 * AGEN Register mask
 */
typedef enum {
    AGEN_MASK_DEVID  = 0xf0,    // Device Id number
    AGEN_MASK_MUT    = 0x0f,    // Mutation number of control bank
    AGEN_SHIFT_DEVID = 0x4      // Shift value for dev id
} as7331_agen_reg_mask_t;

/**
 * Integration times (bits 3:0 of CREG1)
 */
typedef enum {
    INTEGRATION_TIME_1MS,
    INTEGRATION_TIME_2MS,
    INTEGRATION_TIME_4MS,
    INTEGRATION_TIME_8MS,
    INTEGRATION_TIME_16MS,
    INTEGRATION_TIME_32MS,
    INTEGRATION_TIME_64MS,
    INTEGRATION_TIME_128MS,
    INTEGRATION_TIME_256MS,
    INTEGRATION_TIME_512MS,
    INTEGRATION_TIME_1024MS,
    INTEGRATION_TIME_2048MS,
    INTEGRATION_TIME_4096MS,
    INTEGRATION_TIME_8192MS,
    INTEGRATION_TIME_16384MS
} as7331_integration_time_t;

/**
 * Gain settings (bits 7:4 CREG1)
 */
typedef enum {
    GAIN_2048X,
    GAIN_1024X,
    GAIN_512X,
    GAIN_256X, 
    GAIN_128X, 
    GAIN_64X,  
    GAIN_32X,  
    GAIN_16X,  
    GAIN_8X,  
    GAIN_4X,   
    GAIN_2X,   
    GAIN_1X  
} as7331_gain_t;

/**
 * Measurement mode settings (bits 7:6 of CREG3)
 */

typedef enum {
    MEASUREMENT_MODE_CONTINUOUS = 0x00,
    MEASUREMENT_MODE_COMMAND = 0x40,
    MEASUREMENT_MODE_SYNC_START = 0x80 ,
    MEASUREMENT_MODE_SYNC_START_AND_END = 0xc0
} as7331_measure_mode_t;

/**
 * Inernal Clock Frequency
 */
typedef enum {
    CCLK_FREQ_1024KHZ,
    CCLK_FREQ_2048KHZ,
    CCLK_FREQ_4096KHZ,
    CCLK_FREQ_8192KHZ
} as7331_clk_freq_t;

/**
 * structure to indicate status of sensor
 */
typedef struct {
    bool powerstate;
    bool standbystate; 
    bool notready;     
    bool ndata;        
    bool ldata;        
    bool adcof;        
    bool mresof;       
    bool outconvof;
}as7331_status_t;

/**
 * structure to store different settings of AS7331
 */
typedef struct {
    bool power_mode;
    bool measurement_mode;
    bool standby_state;
    bool divider_enable;
    as7331_clk_freq_t cclk;
    as7331_integration_time_t integration_time;
    as7331_gain_t gain;
    int divider;
} as7331_settings_t;

/**
 * AS7331 device data/i2c data structure
 */
typedef struct {
    i2c_dev_t i2c_dev;
    as7331_settings_t settings;
    uint8_t chip_id;
    as7331_status_t status;
} as7331_t;

/**
 * structure to store raw uv radiation and temperature values
 */
typedef struct {
    float uva_raw;
    float uvb_raw;
    float uvc_raw;
    uint16_t temp_raw;
} as7331_raw_values_t;

/**
 * final uv radiation, index and temperature values
 */
typedef struct {
    float uv_a;
    float uv_b;
    float uv_c;
    float uv_index;
    float temperature;
} as7331_values_float_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param addr AS7331 address
 * @param port I2C port number
 * @param sda_gpio GPIO pin for SDA
 * @param scl_gpio GPIO pin for SCL
 * @return `ESP_OK` on success
 */
esp_err_t as7331_init_desc(as7331_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t as7331_free_desc(as7331_t *dev);

/**
 * @brief   Initialize a AS7331 sensor
 *
 * The function initializes the sensor device data structure, probes the
 * sensor, soft resets the sensor, and configures the sensor with the
 * the following default settings:
 *
 * - Oversampling rate for temperature, pressure, humidity is osr_1x
 * - Filter size for pressure and temperature is iir_size 3
 * - Heater profile 0 with 320 degree C and 150 ms duration
 *
 * The sensor must be connected to an I2C bus.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t as7331_init_sensor(as7331_t *dev);

/**
 * @brief   Perform a software reset of the AS7331 sensor
 *
 * This function issues a software reset command to the AS7331 by setting
 * the corresponding bit in the Operational State Register (OSR).
 *
 * After calling this function, the device requires a short delay before
 * it is ready to accept further configuration commands.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t as7331_software_reset(as7331_t *dev);

/**
 * @brief   Read the AS7331 chip ID
 *
 * This function reads the device identification field from the AGEN
 * register and returns the raw value to the caller.
 *
 * The chip ID can be used to verify that the connected device is an
 * AS7331 sensor.
 *
 * @param dev   Device descriptor
 * @param value Pointer to store the chip ID value
 * @return `ESP_OK` on success
 */
esp_err_t as7331_get_chip_id(as7331_t * dev, uint8_t *value);

/**
 * @brief   Set the measurement mode of the AS7331
 *
 * This function configures the measurement mode of the sensor, such as
 * continuous mode or command (one-shot) mode, by updating the appropriate
 * bits in Configuration Register 3 (CREG3).
 *
 * The device is temporarily placed into configuration mode during this
 * operation and returned to measurement mode afterward.
 *
 * @param dev   Device descriptor
 * @param value Measurement mode value
 * @return `ESP_OK` on success
 */
esp_err_t as7331_set_measurement_mode(as7331_t *dev, uint8_t value);

/**
 * @brief   Get the current measurement mode
 *
 * This function reads the measurement mode bits from Configuration
 * Register 3 (CREG3) and returns the current mode.
 *
 * @param dev   Device descriptor
 * @param value Pointer to store the measurement mode value
 * @return `ESP_OK` on success
 */
esp_err_t as7331_get_measurement_mode(as7331_t *dev, uint8_t *value);

/**
 * @brief   Set the integration time
 *
 * This function sets the integration time for UV measurements by writing
 * the corresponding field in Configuration Register 1 (CREG1).
 *
 * Longer integration times increase sensitivity but also increase the
 * measurement duration.
 *
 * @param dev   Device descriptor
 * @param value Integration time setting
 * @return `ESP_OK` on success
 */
esp_err_t as7331_set_integration_time(as7331_t *dev, uint8_t value);

/**
 * @brief   Get the current integration time
 *
 * This function reads the integration time field from Configuration
 * Register 1 (CREG1).
 *
 * @param dev   Device descriptor
 * @param value Pointer to store the integration time value
 * @return `ESP_OK` on success
 */
esp_err_t as7331_get_integration_time(as7331_t *dev, uint8_t *value);

/**
 * @brief   Set the gain value
 *
 * This function configures the analog gain applied to the UV measurement
 * channels by updating the gain field in Configuration Register 1 (CREG1).
 * 
 * Can be set to one of the following values: '2048x', '1024x', '512x',
 * '256x', '128x', '64x', '32x', '16x', '8x ', '4x', '2x', '1x'
 *
 * @param dev   Device descriptor
 * @param value Gain setting
 * @return `ESP_OK` on success
 */
esp_err_t as7331_set_gain(as7331_t *dev, uint8_t value);

/**
 * @brief   Get the current gain value
 *
 * This function reads the gain configuration field from Configuration
 * Register 1 (CREG1).
 *
 * @param dev   Device descriptor
 * @param value Pointer to store the gain value
 * @return `ESP_OK` on success
 */
esp_err_t as7331_get_gain(as7331_t *dev, uint8_t *value);

/**
 * @brief   Enable or disable standby mode
 *
 * This function controls the standby state of the sensor by updating
 * the standby bit in Configuration Register 3 (CREG3).
 *
 * When standby mode is enabled, the sensor reduces power consumption.
 * 
 * Can be set to either True or False
 *
 * @param dev   Device descriptor
 * @param value Standby state (true to enable, false to disable)
 * @return `ESP_OK` on success
 */
esp_err_t as7331_set_standby_state(as7331_t *dev, bool value);

/**
 * @brief   Get the current standby state
 *
 * This function reads the standby state bit from Configuration Register 3
 * (CREG3).
 *
 * @param dev   Device descriptor
 * @param value Pointer to store the standby state value
 * @return `ESP_OK` on success
 */
esp_err_t as7331_get_standby_state(as7331_t *dev, uint8_t *value);

/**
 * @brief   Enable or disable power-down mode
 *
 * This function controls the power-down state of the AS7331 by setting or
 * clearing the corresponding bit in the Operational State Register (OSR).
 *
 * @param dev   Device descriptor
 * @param value Power-down state (true to enable, false to disable)
 * @return `ESP_OK` on success
 */
esp_err_t as7331_set_power_mode(as7331_t *dev, bool value);

/**
 * @brief   Get the current power-down state
 *
 * This function reads the power-down state bit from the Operational State
 * Register (OSR).
 * 
 * 1 -> Power down mode on
 * 2 -> Power down mode off
 *
 * @param dev   Device descriptor
 * @param value Pointer to store the power-down state value
 * @return `ESP_OK` on success
 */
esp_err_t as7331_get_power_mode(as7331_t *dev, uint8_t *value);

/**
 * @brief   Enable or disable the measurement result divider
 *
 * This function enables or disables the digital divider applied to the
 * measurement results by updating Configuration Register 2 (CREG2).
 * 
 * 1 -> Power down mode on
 * 2 -> Power down mode off
 *
 * @param dev   Device descriptor
 * @param value Divider enable state
 * @return `ESP_OK` on success
 */
esp_err_t as7331_set_divider_enabled(as7331_t *dev, bool value);

/**
 * @brief   Get the measurement result divider enable state
 *
 * This function reads the divider enable bit from Configuration
 * Register 2 (CREG2).
 *
 * @param dev   Device descriptor
 * @param value Pointer to store the divider enable state
 * @return `ESP_OK` on success
 */
esp_err_t as7331_get_divider_enabled(as7331_t *dev, uint8_t *value);

/**
 * @brief   Set the measurement result divider value
 *
 * This function configures the digital divider applied to the measurement
 * results by updating the divider field in Configuration Register 2 (CREG2).
 *
 * @param dev   Device descriptor
 * @param value Divider value
 * @return `ESP_OK` on success
 */
esp_err_t as7331_set_divider(as7331_t *dev, uint8_t value);

/**
 * @brief   Get the current measurement result divider value
 *
 * This function reads the divider configuration field from Configuration
 * Register 2 (CREG2).
 *
 * @param dev   Device descriptor
 * @param value Pointer to store the divider value
 * @return `ESP_OK` on success
 */
esp_err_t as7331_get_divider(as7331_t *dev, uint8_t *value);

/**
 * @brief   Set the internal clock frequency
 *
 * This function configures the internal clock frequency of the AS7331
 * by updating the corresponding field in Configuration Register 3 (CREG3).
 *
 * @param dev   Device descriptor
 * @param value Clock frequency setting
 * @return `ESP_OK` on success
 */
esp_err_t as7331_set_cclk(as7331_t *dev, uint8_t value);

/**
 * @brief   Get the current internal clock frequency
 *
 * This function reads the internal clock frequency configuration field
 * from Configuration Register 3 (CREG3).
 *
 * @param dev   Device descriptor
 * @param value Pointer to store the clock frequency value
 * @return `ESP_OK` on success
 */
esp_err_t as7331_get_cclk(as7331_t *dev, uint8_t *value);

/**
 * @brief   Apply the default configuration to the AS7331
 *
 * This function configures the sensor with a predefined set of default
 * parameters, including measurement mode, integration time, gain,
 * divider state, standby state, power mode, and internal clock frequency.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t as7331_set_default_config(as7331_t *dev);

/**
 * @brief   Read UV measurements and convert them to physical values
 *
 * This function performs a UV measurement, reads the raw channel data,
 * converts the results to physical irradiance values, computes the UV
 * index, and converts the temperature measurement to degrees Celsius.
 *
 * @param dev Device descriptor
 * @param out Pointer to structure to store the converted UV and temperature values
 * @return `ESP_OK` on success
 */
esp_err_t as7331_get_uv_values(as7331_t *dev, as7331_values_float_t *out);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __AS7331_H__ */