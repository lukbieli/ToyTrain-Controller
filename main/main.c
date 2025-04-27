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

void controller_task(void *pvParameters)
{
    uint8_t motor_dir = 0;
    uint8_t motor_speed = 0;
    // Main task
    while (1)
    {
        if(BleDriverCli_IsReady() == true)
        {
            // Read ADC value
            int16_t adc_value = adc_read_potentiometerRaw();
            // printf("ADC Value: %d\n", adc_value);
            float adc_voltage = adc_read_batteryVoltage();
            int16_t adc_bat = adc_read_batteryRaw();
            // printf("ADC Voltage: %.04f volts\n", adc_voltage);

            // Calculate motor speed and direction based on ADC value
            calculate_speed_and_direction(&motor_speed, &motor_dir, adc_value, adc_bat); // Calculate motor speed and direction based on ADC value
            printf("MotSpeed: %d, MotDir: %d AdcVal: %d, AdcBat: %d\n", motor_speed, motor_dir, adc_value, adc_bat);
            // printf("%d %d %d %d\n", motor_speed, motor_dir, adc_value, adc_bat);

            // Send ADC value to BLE
            BleDriverCli_SetMotorSpeed(motor_speed); // Send motor speed via BLE
            BleDriverCli_SetMotorDirection(motor_dir); // Send motor direction via BLE

            //read battery voltage and level via BLE
            float battery_voltage = BleDriverCli_GetBatteryVoltage();
            uint8_t battery_level = BleDriverCli_GetBatteryLevel();
            // printf("Battery Voltage: %.02f V bettery level %d\n", battery_voltage, battery_level);
            // ESP_LOGI("Task", "Motor Speed:%d Direction:%d Voltage:%.02f V Level:%d",adc_value >> 8, motor_dir, battery_voltage, battery_level);
        }
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
