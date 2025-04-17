#ifndef __ADC_DRIVER_H__
#define __ADC_DRIVER_H__
#include "stdio.h"

void ads111x_task(void *pvParameters);
int16_t adc_read_potentiometerRaw(void);
float adc_read_potentiometerVoltage(void);
int16_t adc_read_batteryRaw(void);
float adc_read_batteryVoltage(void);

#endif // __ADC_DRIVER_H__
// ADC_DRIVER_H
