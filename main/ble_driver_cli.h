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

#ifndef BLE_COMM_H
#define BLE_COMM_H

#include <stdint.h>

/**
 * @brief Initializes the BLE driver and starts the BLE task.
 * 
 * This function initializes the BLE driver, registers the GATT client profile,
 * and starts the BLE task for communication with the connected device.
 */
void BleDriverCli_Setup(void);

/**
 * @brief Reads the battery voltage from the connected device.
 * 
 * This function reads the battery voltage from the characteristic of the connected device.
 * 
 * @return The battery voltage in volts.
 */
float BleDriverCli_GetBatteryVoltage(void);

/**
 * @brief Reads the battery level from the connected device.
 * 
 * This function reads the battery level from the characteristic of the connected device.
 * 
 * @return The battery level as a percentage (0-100).
 */
uint8_t BleDriverCli_GetBatteryLevel(void);

/**
 * @brief Sends the motor speed to the connected device.
 * 
 * This function sends the motor speed to the characteristic of the connected device.
 * 
 * @param speed The motor speed to be sent (0-255).
 * 
 * @return true if the speed was sent successfully, false otherwise.
 */
bool BleDriverCli_SetMotorSpeed(uint8_t speed);

/**
 * @brief Sends the motor direction to the connected device.
 * 
 * This function sends the motor direction to the characteristic of the connected device.
 * 
 * @param direction The motor direction to be sent (0 for forward, 1 for backward).
 * 
 * @return true if the direction was sent successfully, false otherwise.
 */
bool BleDriverCli_SetMotorDirection(uint8_t direction);

/**
 * @brief Returns boolean value indicating if the connection is ready.
 * 
 * This function checks if the connection to the remote device is established and ready for communication.
 * 
 * @return true if the connection is ready, false otherwise.
 */
bool BleDriverCli_IsReady(void);

#endif // BLE_COMM_H
