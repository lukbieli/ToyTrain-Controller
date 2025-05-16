#ifndef LED_RGB_DRIVER_H
#define LED_RGB_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Steering single RGB LED using PWM for 3 channels D21, D22, D23 using LEDC driver
 */

 /**
  * @brief Initialize the LED driver
  * 
  * This function initializes the LED driver, sets up the GPIO pins, and configures the LEDC driver.
  * It should be called before using any other LED driver functions.
  */
void LedRgbDriver_Init(void);

/**
 * @brief Task to control the LED driver
 * 
 * This function runs in a FreeRTOS task and handles the LED driver operations.
 * It should be called in a separate task to avoid blocking the main application.
 */
void LedRgbDriver_Task(void);

/**
 * @brief Set the color of the LED using RGB values
 * 
 * This function sets the color of the LED using the specified red, green, and blue values.
 * The values should be in the range of 0 to 255.
 * 
 * @param red The red component (0-255)
 * @param green The green component (0-255)
 * @param blue The blue component (0-255)
 */
void LedRgbDriver_SetColorRaw(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Set the brightness of the LED
 * 
 * This function sets the brightness of the LED using a value from 0 to 255.
 * 
 * @param brightness The brightness value (0-255)
 * 
 */
void LedRgbDriver_SetBrightness(uint8_t brightness);

/**
 * @brief Enables blinking of the LED
 * 
 * This function enables blinking of the LED with the specified periodicity.
 * 
 * @param periodicy The periodicity of the blinking in milliseconds
 */
void LedRgbDriver_Blink(uint32_t periodicy);

/**
 * @brief Stops blinking of the LED
 * 
 * This function stops the blinking of the LED and turns it off.
 */
void LedRgbDriver_BlinkStop(void);

/**
 * @brief Turns the LED off
 * 
 */
void LedRgbDriver_Off(void);

#endif // LED_RGB_DRIVER_H
