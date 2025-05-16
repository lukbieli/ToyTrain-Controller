#include "led_rgb_driver.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define LEDC_OUTPUT_RED         GPIO_NUM_25
#define LEDC_OUTPUT_GREEN       GPIO_NUM_26
#define LEDC_OUTPUT_BLUE        GPIO_NUM_27
#define LEDC_CHANNEL_RED        LEDC_CHANNEL_0
#define LEDC_CHANNEL_GREEN      LEDC_CHANNEL_1
#define LEDC_CHANNEL_BLUE       LEDC_CHANNEL_2
#define LEDC_FREQUENCY          5000 // 5 kHz

static uint8_t brightness = 255;
static bool is_blinking = false;
static uint32_t blink_periodicity = 0;
static uint8_t current_red = 0, current_green = 0, current_blue = 0;
static uint32_t timer = 0;
static uint8_t blink_state = 0;

static void configure_ledc_channel(ledc_channel_config_t *channel_config, int gpio_num, ledc_channel_t channel) {
    channel_config->gpio_num = gpio_num;
    channel_config->speed_mode = LEDC_MODE;
    channel_config->channel = channel;
    channel_config->timer_sel = LEDC_TIMER;
    channel_config->duty = 0;
    channel_config->hpoint = 0;
    channel_config->intr_type = LEDC_INTR_DISABLE;
    channel_config->flags.output_invert = 0;
    channel_config->sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;

    ledc_channel_config(channel_config);
}

void LedRgbDriver_Init(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t channel_config;
    configure_ledc_channel(&channel_config, LEDC_OUTPUT_RED, LEDC_CHANNEL_RED);
    configure_ledc_channel(&channel_config, LEDC_OUTPUT_GREEN, LEDC_CHANNEL_GREEN);
    configure_ledc_channel(&channel_config, LEDC_OUTPUT_BLUE, LEDC_CHANNEL_BLUE);
}

static void update_ledc_duty(uint8_t red, uint8_t green, uint8_t blue) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED, (red * brightness) / 255);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RED);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN, (green * brightness) / 255);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GREEN);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BLUE, (blue * brightness) / 255);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BLUE);
}

void LedRgbDriver_SetColorRaw(uint8_t red, uint8_t green, uint8_t blue) {
    current_red = red;
    current_green = green;
    current_blue = blue;

    if (!is_blinking) {
        update_ledc_duty(red, green, blue);
    }
}

void LedRgbDriver_SetBrightness(uint8_t new_brightness) {
    brightness = new_brightness;
    LedRgbDriver_SetColorRaw(current_red, current_green, current_blue);
}

void LedRgbDriver_Blink(uint32_t periodicity) {
    is_blinking = true;
    blink_periodicity = periodicity;
}

void LedRgbDriver_BlinkStop(void) {
    is_blinking = false;
    LedRgbDriver_SetColorRaw(current_red, current_green, current_blue);
}

void LedRgbDriver_Off(void) {
    is_blinking = false;
    LedRgbDriver_SetColorRaw(0, 0, 0);
}

//called with 50ms period
void LedRgbDriver_Task(void) {

    if (is_blinking) {
        timer++;
        if (timer >= (blink_periodicity / 2) / 50) {
            timer = 0;

            if (blink_state == 0) {
                // ESP_LOGI("LED", "Turning on LED");
                update_ledc_duty(current_red, current_green, current_blue);
            } else {
                // ESP_LOGI("LED", "Turning off LED");
                update_ledc_duty(0, 0, 0);
            }
            blink_state = !blink_state;
        }

    }

}
