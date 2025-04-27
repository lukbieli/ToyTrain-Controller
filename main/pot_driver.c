#include "pot_driver.h"

#include "stdint.h"
#include "stdio.h"
#include "math.h"

/**
 * 
 * Potenciometer is going to define motor speed and direction based only on ADC value.
 * 
 * Range of potenciometer is divided in to 2 zones: 
 * 1. Forward zone (DeadZone - 4095) - motor speed is positive and motor direction is forward
 * 2. Backward zone (0 - DeadZone) - motor speed is positive but calculated from deadzone and motor direction is backward
 * 3. Dead zone - motor speed is 0 and motor direction is stopped
 * 
 * |-----------------------|------------|----------------------|
 * |                       |            |                      |
 * |   Backward zone       | Dead zone  |      Forward zone    |
 * SPEED
 * |                       |            |                      |
 * 100%                   0%            0%                    100%
 * |                       |            |                      |
 * ADC
 * |                       |            |                      |
 * 0             (DzMid-DzSize/2)  (DzMid+DzSize/2)          adc_bat
 * |-----------------------|------------|----------------------|
 * 
 * Max pot value is adapting to given adc_bat which is real max value of potentiometer.
 * Dead zone mid is claculated as middle of the adc_bat value.
 */

#define POT_DEAD_ZONE_SIZE (10) // Dead zone size for motor speed adjustment in percentage


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
void calculate_speed_and_direction(uint8_t *motor_speed, uint8_t *motor_dir,int16_t adc_val, int16_t adc_bat)
{
    int16_t adc_val_org = adc_val; // Store original ADC value for debugging
    float speed = 0.0; // Initialize speed variable
    if(adc_val < 0) {
        adc_val = 0; // Ensure ADC value is not negative
    }
    if(adc_val > adc_bat) {
        adc_val = adc_bat; // Ensure ADC value does not exceed battery ADC value
    }
    if(adc_bat <= 0) {
        adc_bat = 1; // Ensure battery ADC value is not zero to avoid division by zero
    }

    // Calculate the dead zone mid value based on the battery ADC value
    int16_t dead_zone_mid = adc_bat / 2; // Middle of the ADC value range
    int32_t dead_zone_size = adc_bat * POT_DEAD_ZONE_SIZE / 100; // Size of the dead zone in ADC value

    // Calculate the dead zone limits
    int16_t dead_zone_min = dead_zone_mid - (dead_zone_size / 2); // Lower limit of the dead zone
    int16_t dead_zone_max = dead_zone_mid + (dead_zone_size / 2); // Upper limit of the dead zone    

    // Check if the ADC value is within the forward zone
    if (adc_val > dead_zone_max) {
        speed = (float)((adc_val - dead_zone_max) * 100.0 / (adc_bat - dead_zone_max)); // Calculate motor speed as a percentage
        *motor_dir = 1; // Forward direction
    }
    // Check if the ADC value is within the backward zone
    else if (adc_val < dead_zone_min) {
        speed = (float)((dead_zone_min - adc_val) * 100.0 / (dead_zone_min)); // Calculate motor speed as a percentage
        *motor_dir = 0; // Backward direction
    }
    // If the ADC value is within the dead zone, set speed to 0 and direction to stopped
    else {
        *motor_dir = 0;   // Stopped direction
        *motor_speed = 0; // Set motor speed to 0
    }

    if(speed > 0.1) {

        // Scale motor speed to [0, 100] based on the calculated nonlinearity of the potenciometer
        if(speed < 90.0)
        {
            *motor_speed = (uint8_t)(speed * 5.0/9.0); // Scale motor speed to [0, 100]
        }
        else
        {
            *motor_speed = (uint8_t)((speed - 80.0) * 5.0); // Scale motor speed to [0, 100]
        }
    }

    printf("%.2f %d %d %d %d %d %d\n", speed, adc_val, adc_bat, dead_zone_min, dead_zone_max, *motor_speed, *motor_dir);
}
