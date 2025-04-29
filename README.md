# Toy Train Controller

This project is designed to control a toy train using an ESP32 microcontroller. It utilizes BLE (Bluetooth Low Energy) for communication, ADC (Analog-to-Digital Converter) for reading potentiometer values, and motor control logic to adjust the train's speed and direction.

## Features
- **BLE Communication**:
  - Read battery voltage and level via BLE.
  - Control motor speed and direction via BLE.
- **ADC Integration**:
  - Reads potentiometer values to calculate motor speed and direction.
  - Dynamically adjusts motor behavior based on battery voltage.
- **State Machine**:
  - Implements a state machine with states: `INIT`, `IDLE`, `WORKING`, and `ERROR`.
- **Motor Control**:
  - Supports forward and reverse motor directions.
  - Includes a dead zone to prevent unintentional motor movement.

## How to Use
1. **Hardware Setup**:
   - Connect a linear potentiometer to the ADC pin of the ESP32.
   - Connect the motor driver to the appropriate GPIO pins.
   - Ensure the ESP32 is powered and connected to the toy train hardware.

2. **Build and Flash**:
   - Use the following commands to build and flash the project:
     ```bash
     idf.py build
     idf.py flash
     ```
   - Monitor the output using:
     ```bash
     idf.py monitor
     ```

3. **BLE Communication**:
   - Use a BLE-compatible device (e.g., smartphone) to connect to the ESP32.
   - Read battery voltage and level or send motor control commands via BLE.

## Project Structure

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   ├── main.c                  # Main application logic
│   ├── pot_driver.c            # Potentiometer and ADC handling
│   ├── ble_driver_cli.c        # BLE communication logic
│   └── adc_driver.c            # ADC driver for reading raw values
└── README.md                   # Project documentation
```

## Key Components
- **`main.c`**:
  - Implements the state machine and main control logic.
- **`pot_driver.c`**:
  - Handles potentiometer readings and calculates motor speed and direction.
- **`ble_driver_cli.c`**:
  - Manages BLE communication for battery monitoring and motor control.
- **`adc_driver.c`**:
  - Provides low-level ADC functionality.

## Dependencies
- **ESP-IDF**:
  - Ensure ESP-IDF v5.4 or later is installed.
  - Follow the [ESP-IDF setup guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) to configure your environment.

## Example Output
```
MotSpeed: 50, MotDir: 1, AdcVal: 20000, AdcBat: 26592
Battery Voltage: 3.70 V, Battery Level: 85%
```

## Future Improvements
- Add support for additional BLE characteristics.
- Implement advanced motor control algorithms.
- Optimize ADC reading and processing for better performance.

## License
This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.
