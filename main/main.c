#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "adc_driver.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include <string.h>
#include "i2cdev.h"
#include "ble_comm.h"

void controller_task(void *pvParameters)
{
    uint8_t motor_dir = 0;
    // Main task
    while (1)
    {
        // Read ADC value
        int16_t adc_value = adc_read_potentiometerRaw();
        printf("ADC Value: %d\n", adc_value);
        float adc_voltage = adc_read_batteryVoltage();
        printf("ADC Voltage: %.04f volts\n", adc_voltage);
        send_motor_speed((uint8_t)(adc_value >> 8)); // Send motor speed via BLE
        motor_dir ^= 1; // Toggle motor direction
        send_motor_direction(motor_dir); // Send motor direction via BLE
        vTaskDelay(pdMS_TO_TICKS(3000)); // Delay for 3 seconds

        //read battery voltage and level via BLE
        float battery_voltage = read_battery_voltage();
        uint8_t battery_level = read_battery_level();
        printf("Battery Voltage: %.02f V bettery level %d\n", battery_voltage, battery_level);
        vTaskDelay(pdMS_TO_TICKS(3000));
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
    ble_task();
}
