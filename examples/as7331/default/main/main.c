#include <stdio.h>
#include <as7331.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <string.h>

#define PORT 0
#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_0)
#define ADDR AS7331_I2C_ADDR_0
#endif
#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_1)
#define ADDR AS7331_I2C_ADDR_1
#endif
#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_2)
#define ADDR AS7331_I2C_ADDR_2
#endif
#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_3)
#define ADDR AS7331_I2C_ADDR_3
#endif

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

void as7331_test(void *pvParameters)
{
    as7331_t sensor;
    memset(&sensor, 0, sizeof(as7331_t));

    ESP_ERROR_CHECK(as7331_init_desc(&sensor, ADDR, PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // init the sensor
    ESP_ERROR_CHECK(as7331_init_sensor(&sensor));

    // initialize sensor
    // does software reset and set default config
    as7331_init_sensor(&sensor);

    // structure to store sensor output
    as7331_values_float_t values;
    while (1)
    {
        // take measurement
        if (as7331_get_uv_values(&sensor, &values) == ESP_OK)
        {
            printf("AS7331 Sensor: UV A: %.2f , UV B: %.2f , UV C: %.2f , UV Index: %.2f, Temperature: %.2f °C\n",
                        values.uv_a, values.uv_b, values.uv_c, values.uv_index, values.temperature);
        }
        // waiting for 3 second
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreatePinnedToCore(as7331_test, "as7331_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}
