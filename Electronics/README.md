# A.M.O.A.N Electronics Subsystem

This repository contains all documentation and resources related to the electronics subsystem of the A.M.O.A.N (Autonomous Monitoring of Avian Nesting) system. The subsystem is responsible for acquiring, processing, and transmitting weight data from Cape Cormorants in a field-deployable, low-power setup.

## Repository Contents

- **Context Diagram**: High-level overview of how the electronics subsystem interacts with other parts of the system.
- **Wiring Diagram**: Complete schematic showing connections between the ESP32, HX711, load cell, and power components.
- **Program Test Examples**: Firmware for testing the electronics, based on ESP32 example sketches. Includes:
  - Load cell calibration
  - BLE and ESP-NOW communication
  - SPIFFS-based data logging
- **Electronics Enclosure**:
  - STL files for 3D printing the custom electronics enclosure
  - Technical drawings for mechanical reference
  - STEP file of enclosure
- **Datasheets**:
  - ESP32 Dev Board
  - HX711 Load Cell Amplifier
  - MP1584 Buck Converter
  - Load Cell specifications
  -Alternative Component datasheets

## Getting Started

1. Clone the repository and open the test software folder in the [Arduino IDE](https://www.arduino.cc/en/software) or [PlatformIO](https://platformio.org/).
2. Connect your ESP32 to your computer via USB-C.
3. Upload the test firmware for verifying sensor readings and communications.
4. Refer to the wiring diagram before connecting hardware.
5. Use the STL files to 3D print the enclosure or modify the provided drawings to suit your mounting needs.

## License

This repository is provided for academic and research use. Please contact the project maintainers before commercial use or redistribution.

