# MICRO-ROS FOR MOBILE ROBOTICS SYSTEMS

Project to put micro-ROS into ABB's mobile YuMi platform (mYuMi), conducted as a 30 hp master's thesis for Mälardalen University.

The micro-ROS setup guides for integration into external tools can be read as standalone for anyone wanting more detailed versions of the existing micro-ROS guides.

## Setup guide

All guides assumes the use of Ubuntu 20.04 as the OS, Galactic as the version for ROS2 and micro-ROS, Fast DDS as the middleware for ROS2, and Micro XRCE-DDS as the middleware for micro-ROS.

An option not explained in this guide is compiling micro-ROS with ROS2 if your board is [supported](https://micro.ros.org/docs/overview/hardware/). Follow the [RTOS guide](https://micro.ros.org/docs/tutorials/core/first_application_rtos/) for this.

### micro-ROS integration into external tools

[STM32Cube](STM32/README.md)

[ESP-IDF](ESP32/README.md)

[Arduino](Arduino/README.md)