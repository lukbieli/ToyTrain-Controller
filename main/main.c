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
#include "adc_driver.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include <string.h>
#include "i2cdev.h"
#include "ble_driver_cli.h"
#include "pot_driver.h"

//state machine states
typedef enum {
    STATE_INIT,
    STATE_IDLE,
    STATE_WORKING,
    STATE_ERROR,
} state_t;

static state_t current_state = STATE_INIT; // Initialize the state machine to the initial state

static state_t state_working(void)
{
    uint8_t motor_dir = 0;
    uint8_t motor_speed = 0;

    // previous values
    static uint8_t prev_motor_speed = 0;
    static uint8_t prev_motor_dir = 0;

    // Main task
    // Read ADC value
    int16_t adc_value = adc_read_potentiometerRaw();
    // printf("ADC Value: %d\n", adc_value);
    float adc_voltage = adc_read_batteryVoltage();
    int16_t adc_bat = adc_read_batteryRaw();
    // printf("ADC Voltage: %.04f volts\n", adc_voltage);

    // Calculate motor speed and direction based on ADC value
    calculate_speed_and_direction(&motor_speed, &motor_dir, adc_value, adc_bat); // Calculate motor speed and direction based on ADC value
    //if motor_speed, motor_dir, adc_value, adc_bat are same as previous values, skip sending to BLE
    if(motor_speed != prev_motor_speed) {

        
        if(BleDriverCli_SetMotorSpeed(motor_speed) == true) // Send motor speed via BLE
        {
            printf("MotSpeed: %d, MotDir: %d AdcVal: %d, AdcBat: %d\n", motor_speed, motor_dir, adc_value, adc_bat);
            // printf("%d %d %d %d\n", motor_speed, motor_dir, adc_value, adc_bat);

            // Update previous values
            prev_motor_speed = motor_speed;
        }
    }

    if(motor_dir != prev_motor_dir) {

        if( BleDriverCli_SetMotorDirection(motor_dir) == true) // Send motor direction via BLE
        {
            prev_motor_dir = motor_dir;
        }
    }


    //read battery voltage and level via BLE
    float battery_voltage = BleDriverCli_GetBatteryVoltage();
    uint8_t battery_level = BleDriverCli_GetBatteryLevel();
    // printf("Battery Voltage: %.02f V bettery level %d\n", battery_voltage, battery_level);
    // ESP_LOGI("Task", "Motor Speed:%d Direction:%d Voltage:%.02f V Level:%d",adc_value >> 8, motor_dir, battery_voltage, battery_level);


    return STATE_WORKING; // Return to the working state
}

static state_t state_machine(state_t state)
{
    switch (state)
    {
        case STATE_INIT:
            // Initialize the state machine
            state = STATE_IDLE; // Transition to the idle state
            break;

        case STATE_IDLE:
            // Idle state, waiting for connection to ble device
            if (BleDriverCli_IsReady() == true)
            {
                ESP_LOGI("Task", "BLE device is ready, starting working state...");
                state = STATE_WORKING; // Transition to the working state
            }
            break;

        case STATE_WORKING:
            // Working state
            state = state_working(); // Call the working function

            if(BleDriverCli_IsReady() == false)
            {
                ESP_LOGI("Task", "BLE device is not ready, returning to idle state...");
                state = STATE_IDLE; // Transition to the idle state if not ready
            }
            break;

        case STATE_ERROR:
            // Error state, handle errors
            break;

        default:
            // Invalid state
            break;
    }

    return state;
}

void controller_task(void *pvParameters)
{

    while (1)
    {
        // Call the state machine function
        current_state = state_machine(current_state);
        vTaskDelay(pdMS_TO_TICKS(50)); // Delay for 20ms
    }
}

void app_main(void)
{
    // Init library
    ESP_ERROR_CHECK(i2cdev_init());


    // Start task
    xTaskCreatePinnedToCore(ads111x_task, "ads111x_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);

    // main task
    xTaskCreatePinnedToCore(controller_task, "controller_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);

    // Start BLE task
    BleDriverCli_Setup();
}
