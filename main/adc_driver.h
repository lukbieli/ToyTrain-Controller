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

#ifndef __ADC_DRIVER_H__
#define __ADC_DRIVER_H__
#include "stdio.h"

void ads111x_task(void *pvParameters);
int16_t adc_read_potentiometerRaw(void);
float adc_read_potentiometerVoltage(void);
int16_t adc_read_batteryRaw(void);
float adc_read_batteryVoltage(void);
int16_t adc_read_supplyRaw(void);
float adc_read_supplyVoltage(void);
bool adc_is_data_ready(void);
#endif // __ADC_DRIVER_H__
// ADC_DRIVER_H
