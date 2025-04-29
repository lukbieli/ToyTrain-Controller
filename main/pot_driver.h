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

#ifndef POT_DRIVER_H
#define POT_DRIVER_H

#include <stdint.h>

/**
 * * @brief Calculates the motor speed and direction based on ADC values.
 * 
 * * This function takes the ADC value and battery voltage as input and calculates the motor speed and direction.
 * * The motor speed and the direction is determined by the ADC value.
 * 
 * * @param motor_speed Pointer to the variable where the calculated motor speed will be stored.
 * * @param motor_dir Pointer to the variable where the calculated motor direction will be stored.
 * * @param adc_val The ADC value read from the potentiometer.
 * * @param adc_bat The ADC value read from the battery voltage.
 * 
 * * @return None
 */
void calculate_speed_and_direction(uint8_t *motor_speed, uint8_t *motor_dir,int16_t adc_val, int16_t adc_bat);

#endif // POT_DRIVER_H
