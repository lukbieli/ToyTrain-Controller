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
    // Main task
    while (1)
    {
        // Read ADC value
        int16_t adc_value = adc_read_potentiometerRaw();
        printf("ADC Value: %d\n", adc_value);
        float adc_voltage = adc_read_batteryVoltage();
        printf("ADC Voltage: %.04f volts\n", adc_voltage);
        // Send data via BLE
        send_data_via_ble(adc_value);

        uint16_t data = read_data_via_ble();
        printf("Data from BLE: %d\n", data);
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
