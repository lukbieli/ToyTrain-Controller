// single_led_driver.c
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_single_driver.h"
#include "esp_log.h"

#define SINGLE_LED_GPIO GPIO_NUM_33
#define SINGLE_LED_CHANNEL LEDC_CHANNEL_3
#define SINGLE_LED_FREQUENCY 5000 // 5 kHz

static uint8_t single_led_brightness = 255;
static bool is_blinking = false;
static uint32_t blink_periodicity = 0;
static uint32_t timer = 0;
static uint8_t blink_state = 0;

void LedSingleDriver_Init(void) {
    ledc_channel_config_t channel_config = {
        .gpio_num = SINGLE_LED_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = SINGLE_LED_CHANNEL,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE
    };
    ledc_channel_config(&channel_config);
}

//task called in a loop 50 ms
void LedSingleDriver_Task(void) {
    if(is_blinking) {
        timer++;
        if(timer >= ((blink_periodicity/2) / 50)) {
            timer = 0;
            if(blink_state == 0) {
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, SINGLE_LED_CHANNEL, single_led_brightness);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, SINGLE_LED_CHANNEL);
                blink_state = 1;
            } else {
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, SINGLE_LED_CHANNEL, 0);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, SINGLE_LED_CHANNEL);
                blink_state = 0;
            }
        }
    }
}

void LedSingleDriver_SetBrightness(uint8_t brightness) {
    single_led_brightness = brightness;
    if(!is_blinking) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, SINGLE_LED_CHANNEL, brightness);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, SINGLE_LED_CHANNEL);
    }
}

void LedSingleDriver_Off(void) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, SINGLE_LED_CHANNEL, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, SINGLE_LED_CHANNEL);
}

void LedSingleDriver_On(void) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, SINGLE_LED_CHANNEL, single_led_brightness);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, SINGLE_LED_CHANNEL);
}

void LedSingleDriver_Blink(uint32_t periodicity) {
    is_blinking = true;
    blink_periodicity = periodicity;
    timer = 0;
    blink_state = 0;
    LedSingleDriver_Off(); // Turn off the LED initially
}
