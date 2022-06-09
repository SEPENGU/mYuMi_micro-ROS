# micro-ROS for Arduino

This guide sets up micro-ROS with Arduino, based on the [Arduino guide](https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/README.md) by micro-ROS.

Tested with Arduino Due, ESP32-WROOM-32E, Adafruit HUZZAH32, and NodeMCU-32S.

## Setup

Note: Check first if your board is [supported](https://github.com/micro-ROS/micro_ros_arduino/tree/galactic#supported-boards).

### Arduino IDE

1. Download and install [Arduino IDE](https://www.arduino.cc/en/software).
2. Clone the Galactic version of the [micro-ROS library](https://github.com/micro-ROS/micro_ros_arduino.git).
3. Include the library in Arduino IDE.
4. Patch Arduino board by replacing `platform.txt`:\
    4.1. Teensyduino:
    ```bash
    export ARDUINO_PATH=[Your Arduino + Teensiduino path]
    cd $ARDUINO_PATH/hardware/teensy/avr/
    curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/main/extras/patching_boards/platform_teensy.txt > platform.txt
    ```
    4.2. SAMD:
    ```bash
    export ARDUINO_PATH=[Your Arduino path]
    cd $ARDUINO_PATH/hardware/sam/1.6.12/
    curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/main/extras/patching_boards/platform_arduinocore_sam.txt > platform.txt
    ```
5. Copy the code from the `main.cpp` file, from one of the projects in this folder (Arduino), and paste it into an Arduino sketch.
6. Download the board packages if it doesn't exist and select it (and specify port).
7. Build and flash the project.
8. Start and connect the [Agent](../Agent.md).

### PlatformIO

NEW: Official instructions available in separate [PlatformIO repository](https://github.com/micro-ROS/micro_ros_platformio).

### Troubleshoot

TODO