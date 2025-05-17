/*
 * Copyright 2025 Lukasz Bielinski
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ads111x.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"

#define I2C_PORT 0

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define GAIN ADS111X_GAIN_4V096 // +-4.096V

// Descriptors
static i2c_dev_t device;

// Gain value
static float gain_val;
static int16_t potenciometerValue = 0;
static float potenciometerVoltage = 0;
static int16_t batteryValue = 0;
static float batteryVoltage = 0;
static int16_t supplyValue = 0;
static float supplyVoltage = 0;

static bool data_ready = false;


static void measure_channel(ads111x_mux_t channel, int16_t *result, float* voltage)
{
    // Set the input multiplexer to the desired channel
    ESP_ERROR_CHECK(ads111x_set_input_mux(&device, channel));
    ads111x_start_conversion(&device);
    // Wait for conversion to complete (if needed)
    bool busy;
    do
    {
        ads111x_is_busy(&device, &busy);
    } while (busy);

    // Read the result
    int16_t raw = 0;
    if (ads111x_get_value(&device, &raw) == ESP_OK)
    {
        // Store the value in the provided result pointer
        *voltage = gain_val / ADS111X_MAX_VALUE * raw;
        *result = raw;

        // Debug output (optional)
        // printf("Channel %d: Raw ADC value: %d, voltage: %.04f volts\n", channel, raw, voltage);
    }
    else
    {
        printf("Cannot read ADC value for channel %d\n", channel);
    }
}

int16_t adc_read_potentiometerRaw(void)
{
    // Return ADC value
    return potenciometerValue;
}

float adc_read_potentiometerVoltage(void)
{
    // Return ADC value
    return potenciometerVoltage;
}

int16_t adc_read_batteryRaw(void)
{
    // Return ADC value
    return batteryValue;
}

float adc_read_batteryVoltage(void)
{
    // Return ADC value
    return batteryVoltage;
}

bool adc_is_data_ready(void)
{
    // Return data ready flag
    return data_ready;
}

int16_t adc_read_supplyRaw(void)
{
    // Return ADC value
    return supplyValue;
}

float adc_read_supplyVoltage(void)
{
    // Return ADC value
    return supplyVoltage;
}

// Main task
void ads111x_task(void *pvParameters)
{
    // Clear device descriptors
    memset(&device, 0, sizeof(device));

    gain_val = ads111x_gain_values[GAIN];

    // Setup ICs

    ESP_ERROR_CHECK(ads111x_init_desc(&device, ADS111X_ADDR_GND, I2C_PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    ESP_ERROR_CHECK(ads111x_set_mode(&device, ADS111X_MODE_SINGLE_SHOT));    // Continuous conversion mode
    ESP_ERROR_CHECK(ads111x_set_data_rate(&device, ADS111X_DATA_RATE_32)); // 32 samples per second
    // ESP_ERROR_CHECK(ads111x_set_input_mux(&device, ADS111X_MUX_0_GND));    // positive = AIN0, negative = GND
    ESP_ERROR_CHECK(ads111x_set_gain(&device, GAIN));
    

    while (1)
    {
        // Measure channel 0
        measure_channel(ADS111X_MUX_0_GND, &potenciometerValue, &potenciometerVoltage);

        // Measure channel 1
        measure_channel(ADS111X_MUX_2_GND, &batteryValue, &batteryVoltage);

        measure_channel(ADS111X_MUX_1_GND, &supplyValue, &supplyVoltage);

        data_ready = true;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
