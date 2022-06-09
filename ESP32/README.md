# micro-ROS for ESP-IDF

This guide sets up micro-ROS with ESP-IDF v4.3, based on the [ESP-IDF guide](https://github.com/micro-ROS/micro_ros_espidf_component/blob/galactic/README.md) by micro-ROS.

Tested with ESP32-WROOM-32E, Adafruit HUZZAH32, and NodeMCU-32S.

## Setup

WARNING: Do not download the ESP-IDF prerequisites, this guide will not work if you do.

1. Clone [ESP-IDF v4.3](https://github.com/espressif/esp-idf/tree/release/v4.3) and run `install.sh` to install dependencies.\
    1.1. (Optional) Create an alias to run `export.sh`.
2. Export the environment variables by running `export.sh`, and then to install packages inside the environment for micro-ROS, run:
    ```bash
    pip3 install catkin_pkg lark-parser empy colcon-common-extensions importlib-resources
    ```
3. Open up one of the two projects inside this folder (ESP32).
4. Clone the Galactic version of the [micro-ROS component](https://github.com/micro-ROS/micro_ros_espidf_component/tree/galactic) into the components folder.
5. Put the terminal at the root of the project and run the following commands:

    ```bash
    # ESP-IDF environment variables must be exported first

    # Only needs to be run once per initial build
    idf.py set-target esp32
    idf.py menuconfig # Specify serial or UDP and configure Wi-Fi if used

    # Run each time code has changed
    idf.py build
    idf.py flash

    # To see serial monitor output
    idf.py monitor
    ```

6. Start and connect the [Agent](../Agent.md).

### Troubleshoot

TODO