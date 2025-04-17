#ifndef BLE_COMM_H
#define BLE_COMM_H

void send_data_via_ble(const int16_t data);
void ble_task(void);
uint16_t read_data_via_ble(void);

#endif // BLE_COMM_H
