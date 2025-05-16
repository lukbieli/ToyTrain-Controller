// filepath: e:\Workspace\esp-idf-workspace\toytrain-controller\main\led_single_driver.h
#ifndef LED_SINGLE_DRIVER_H
#define LED_SINGLE_DRIVER_H

#include <stdint.h>

// Initializes the single LED driver
void LedSingleDriver_Init(void);

void LedSingleDriver_Task(void);

// Sets the brightness of the single LED (0-255)
void LedSingleDriver_SetBrightness(uint8_t brightness);

// Turns off the single LED
void LedSingleDriver_Off(void);

void LedSingleDriver_Blink(uint32_t periodicy);

void LedSingleDriver_On(void);

#endif // LED_SINGLE_DRIVER_H
