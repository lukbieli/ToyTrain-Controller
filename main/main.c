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
#include "led_rgb_driver.h"
#include "led_single_driver.h"

#define TIMEOUT_MS (60000) // Timeout in milliseconds
#define UPDATE_MS (1000) // Update time in milliseconds
#define LOOP_TIME_MS (50) // Loop time in milliseconds

//state machine states
typedef enum {
    STATE_INIT,
    STATE_IDLE,
    STATE_WORKING,
    STATE_ERROR,
} state_t;

static state_t current_state = STATE_INIT; // Initialize the state machine to the initial state

static uint32_t timeout_timer = 0; // Timer for timeout
static uint32_t update_timer = 0; // Timer for update
static bool timeout_flag = false; // Flag for timeout

static void test_pot(void){
    uint8_t motor_dir = 0;
    uint8_t motor_speed = 0;

    // previous values
    static uint8_t prev_motor_speed = 0xFF;

    // Main task
    // Read ADC value
    int16_t adc_value = adc_read_potentiometerRaw();
    // printf("ADC Value: %d\n", adc_value);
    // float adc_voltage = adc_read_batteryVoltage();
    int16_t adc_supply = adc_read_supplyRaw();
    // printf("ADC Voltage: %.04f volts\n", adc_voltage);

    // Calculate motor speed and direction based on ADC value
    calculate_speed_and_direction(&motor_speed, &motor_dir, adc_value, adc_supply); // Calculate motor speed and direction based on ADC value
    //if motor_speed, motor_dir, adc_value, adc_bat are same as previous values, skip sending to BLE
    if(motor_speed != prev_motor_speed) {


            printf("MotSpeed: %d, MotDir: %d AdcVal: %d, AdcBat: %d\n", motor_speed, motor_dir, adc_value, adc_supply);
            // printf("%d %d %d %d\n", motor_speed, motor_dir, adc_value, adc_bat);

            // Update previous values
            prev_motor_speed = motor_speed;
        
    }
}

static state_t state_working(void)
{
    uint8_t motor_dir = 0;
    uint8_t motor_speed = 0;

    state_t state = STATE_WORKING; // Set the state to working

    // previous values
    static uint8_t prev_motor_speed = 0xFF;
    static uint8_t prev_motor_dir = 0xFF;

    // Main task
    // Read ADC value
    int16_t adc_value = adc_read_potentiometerRaw();
    // printf("ADC Value: %d\n", adc_value);
    // float adc_voltage = adc_read_batteryVoltage();
    int16_t adc_supply = adc_read_supplyRaw();
    // printf("ADC Voltage: %.04f volts\n", adc_voltage);

    // Calculate motor speed and direction based on ADC value
    calculate_speed_and_direction(&motor_speed, &motor_dir, adc_value, adc_supply); // Calculate motor speed and direction based on ADC value
    //if motor_speed, motor_dir, adc_value, adc_bat are same as previous values, skip sending to BLE
    if(motor_speed != prev_motor_speed) {

        timeout_timer = 0; // Reset timer if motor speed has changed
        update_timer = 0; // Reset timer if motor speed has changed
        timeout_flag = false; // Reset timeout flag if motor speed has changed
        if(BleDriverCli_SetMotorSpeed(motor_speed) == true) // Send motor speed via BLE
        {
            printf("MotSpeed: %d, MotDir: %d AdcVal: %d, AdcBat: %d\n", motor_speed, motor_dir, adc_value, adc_supply);
            // printf("%d %d %d %d\n", motor_speed, motor_dir, adc_value, adc_bat);

            // Update previous values
            prev_motor_speed = motor_speed;
        }
    }
    else
    {   
        if((timeout_flag == false) && (update_timer > (UPDATE_MS/LOOP_TIME_MS))) // Check if update time has passed
        {
            update_timer = 0; // Reset timer
            BleDriverCli_SetMotorSpeed(motor_speed); // Send motor speed via BLE
            ESP_LOGI("Task", "Send update of motor speed: %d", motor_speed);
        }
        else
        {
            update_timer++; // Increment update timer
        }
        // Check if timeout has occurred
        if(timeout_timer > (TIMEOUT_MS/LOOP_TIME_MS)) // Check if timeout has occurred
        {
            timeout_timer = 0; // Reset timer
            timeout_flag = true; // Set timeout flag
            BleDriverCli_SetMotorSpeed(0); // Stop motor if timeout occurs
            ESP_LOGI("Task", "Timeout occurred, stopping motor.");
            // printf("Timeout: %d %d %d %d\n", motor_speed, motor_dir, adc_value, adc_bat);
        }
        else
        {
            timeout_timer++; // Increment timeout timer
        }
    }

    if(motor_dir != prev_motor_dir) {

        if( BleDriverCli_SetMotorDirection(motor_dir) == true) // Send motor direction via BLE
        {
            prev_motor_dir = motor_dir;
        }
    }


    //read battery voltage and level via BLE
    float battery_voltage = 0.0;
    if(BleDriverCli_GetBatteryVoltage(&battery_voltage) == true) // Read battery voltage from the characteristic
    {
        ESP_LOGI("Task", "Battery Voltage: %.02f V\n", battery_voltage);
    }
    //read battery level via BLE
    uint8_t battery_level = 0;
    if(BleDriverCli_GetBatteryLevel(&battery_level) == true) // Read battery level from the characteristic
    {
        ESP_LOGI("Task", "Battery Level: %d%%\n", battery_level);
        if(battery_level < 20)
        {
            // start blinking single LED
            LedSingleDriver_Blink(1000); // Set LED color to white
            LedSingleDriver_SetBrightness(255); // Set LED brightness to maximum
            ESP_LOGI("Task", "Battery level is low, blinking LED.");
        }
        else
        {
            // stop blinking single LED
            LedSingleDriver_Off();
        }
    }


    // ESP_LOGI("Task", "Motor Speed:%d Direction:%d Voltage:%.02f V Level:%d",adc_value >> 8, motor_dir, battery_voltage, battery_level);


    return state; // Return state
}

static state_t battery_monitor(state_t state)
{
    state_t next_state = state;

    if(adc_is_data_ready() == true) // check if adc is ready
    {
        // read out controller battery voltage and level via BLE
        float battery_voltage = adc_read_batteryVoltage() * 2.0; // read battery voltage 

        // 

        uint8_t battery_level = 0;
        if(battery_voltage > 3.6)
        {
            // calculate battery level where 0% is 3.6 and 4.2 is 100%
            battery_level = (battery_voltage - 3.6) / (4.2 - 3.6) * 100;
            if(battery_level > 100)
            {
                battery_level = 100;
            }
        }

        //set rgb to yellow if battery level is below 20%
        if(battery_level < 15)
        {
            LedRgbDriver_SetColorRaw(255, 0,0); // Set LED color to red
            LedRgbDriver_Blink(1000); // Set LED color to white
            next_state = STATE_ERROR; // Set state to error
            ESP_LOGI("Task", "Battery level is critical, blinking LED.");
            printf("Battery Voltage: %.02f V %d%%\n", battery_voltage, battery_level);

        }
        else if( battery_level < 30)
        {
            LedRgbDriver_SetColorRaw(255, 255, 0); // Set LED color to yellow
            ESP_LOGI("Task", "Battery level is low, LED is yellow.");
        }
        else
        {
            //do nothing
        }
    }

    return next_state; // Return the next state
}

static state_t state_machine(state_t state)
{
    state = battery_monitor(state); // Call the battery monitor function

    switch (state)
    {
        case STATE_INIT:
            // Initialize the state machine
            state = STATE_IDLE; // Transition to the idle state
            LedRgbDriver_SetColorRaw(0, 0, 255); // Set LED color to blue
            LedRgbDriver_Blink(600); // Set LED color to blue
            break;

        case STATE_IDLE:
            // Idle state, waiting for connection to ble device
            if (BleDriverCli_IsReady() == true)
            {
                ESP_LOGI("Task", "BLE device is ready, starting working state...");
                state = STATE_WORKING; // Transition to the working state
                LedRgbDriver_SetColorRaw(0, 255, 0); // Set LED color to green
                LedRgbDriver_BlinkStop(); // Stop blinking
            }
            test_pot();
            break;

        case STATE_WORKING:
            // Working state
            state = state_working(); // Call the working function

            if(BleDriverCli_IsReady() == false)
            {
                ESP_LOGI("Task", "BLE device is not ready, returning to idle state...");
                state = STATE_IDLE; // Transition to the idle state if not ready
                LedRgbDriver_SetColorRaw(0, 0, 255); // Set LED color to blue
                LedRgbDriver_Blink(600); // Set LED color to blue
            }
            break;

        case STATE_ERROR:
            // Error state, handle errors
            LedRgbDriver_SetColorRaw(255, 0, 0); // Set LED color to red
            break;

        default:
            // Invalid state
            break;
    }

    return state;
}

void controller_task(void *pvParameters)
{    
    // LedSingleDriver_On(); // Turn on the single LED
    // LedSingleDriver_SetBrightness(255); // Set LED brightness to maximum

    // LedRgbDriver_SetColorRaw(255, 0, 0); // Set LED color to red
    // vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    // LedSingleDriver_SetBrightness(200); 
    // // set color to green
    // LedRgbDriver_SetColorRaw(0, 255, 0); // Set LED color to green
    // vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    // // set color to blue
    // LedRgbDriver_SetColorRaw(0, 0, 255); // Set LED color to blue
    // vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    // // set color to white
    // LedRgbDriver_SetColorRaw(255, 255, 255); // Set LED color to white
    // vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second

    // // enable blinking
    // LedRgbDriver_Blink(100); // Set LED color to white
    // vTaskDelay(pdMS_TO_TICKS(4000)); // Delay for 1 second

    // // stop blinking
    // LedRgbDriver_BlinkStop(); // Set LED color to white
    // vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    // // set color yellow
    // LedRgbDriver_SetColorRaw(255, 255, 0); // Set LED color to yellow
    // vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    // LedRgbDriver_Blink(400); // Set LED color to white

    // LedSingleDriver_SetBrightness(255); // Set LED brightness to maximum
    // LedSingleDriver_Blink(600); // Set LED color to white
    while (1)
    {
        // Call the state machine function
        current_state = state_machine(current_state);
        vTaskDelay(pdMS_TO_TICKS(LOOP_TIME_MS)); // Delay for a short period to avoid busy waiting
    }
}

void led_driver_task(void *pvParameters)
{
    while (1)
    {
        // Call the LED driver task
        LedRgbDriver_Task();
        LedSingleDriver_Task();
        vTaskDelay(pdMS_TO_TICKS(LOOP_TIME_MS)); // Delay for a short period to avoid busy waiting
    }
}

void app_main(void)
{
    // Init library
    ESP_ERROR_CHECK(i2cdev_init());

    // Init LED driver
    LedRgbDriver_Init();
    LedRgbDriver_SetColorRaw(0, 0, 0); // Set LED color to off
    LedRgbDriver_SetBrightness(200); // Set LED brightness

    LedSingleDriver_Init();
    LedSingleDriver_SetBrightness(255); // Set LED brightness to maximum
    LedSingleDriver_Off(); // Turn off the LED



    // Start task
    xTaskCreatePinnedToCore(ads111x_task, "ads111x_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);

    // Start led driver task
    xTaskCreatePinnedToCore(led_driver_task, "led_driver_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);

    // main task
    xTaskCreatePinnedToCore(controller_task, "controller_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);




    // Start BLE task
    BleDriverCli_Setup();
}
