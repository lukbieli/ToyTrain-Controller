#ifndef BLE_COMM_H
#define BLE_COMM_H

void ble_task(void);


float read_battery_voltage(void);
uint8_t read_battery_level(void);
void send_motor_speed(uint8_t speed);
void send_motor_direction(uint8_t direction);

#endif // BLE_COMM_H
