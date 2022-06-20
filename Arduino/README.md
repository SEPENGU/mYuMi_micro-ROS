# micro-ROS for Arduino

This guide sets up micro-ROS with Arduino, based on the [Arduino guide](https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/README.md) by micro-ROS.

Tested with Arduino Due, ESP32-WROOM-32E, Adafruit HUZZAH32, and NodeMCU-32S.

## Setup

Note: Check first if your board is [supported](https://github.com/micro-ROS/micro_ros_arduino/tree/galactic#supported-boards).

### Arduino IDE

1. Download and install [Arduino IDE](https://www.arduino.cc/en/software).
2. Clone the Galactic version of the [micro-ROS library](https://github.com/micro-ROS/micro_ros_arduino.git) inside your Arduino libraries folder, or download it as a ZIP and use the `Add .ZIP library` function in the IDE.
3. If any of the following boards are used, patch them by replacing `platform.txt`:

    4.1. SAMD boards:
    ```bash
    # Locate your terminal at ../hardware/sam/1.6.12/ in your Arduino installation
    curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/main/extras/patching_boards/platform_arduinocore_sam.txt > platform.txt
    ```

    4.2. Teensyduino boards:
    ```bash
    # Locate your terminal at ../hardware/teensy/avr/ in your Arduino installation
    curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/main/extras/patching_boards/platform_teensy.txt > platform.txt
    ```

4. To create a project, either use one of the [examples](https://github.com/micro-ROS/micro_ros_arduino/tree/galactic/examples) included in the library, use one of the projects in this folder (Arduino), or include the following example code for publishers and subscribers, in a sketch:

```C
#include <Arduino.h>

// Standard micro-ROS libraries

#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h> // Example message type for topics

// Standard micro-ROS setup

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// Example declarations for topic communication

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rcl_timer_t timer;
std_msgs__msg__Int32 msg_pub;
std_msgs__msg__Int32 msg_sub;


void publisher_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    msg_pub.data++;
    rcl_publish(&publisher, &msg_pub, NULL);
  }
}

void subscriber_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  printf("Received: %d", msg->data);
}

void setup() {
    set_microros_transports(); // Serial USB
    // set_microros_wifi_transports("WIFI SSID", "WIFI PASS", "IP", PORT);
    
    allocator = rcl_get_default_allocator();

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "mros_arduino_node", "", &support);

    const rosidl_message_type_support_t* type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
    rclc_publisher_init_default(&publisher, &node, type_support, "mros_pub");
    rclc_subscription_init_default(&subscriber, &node, type_support, "mros_sub");

    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), publisher_callback);

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscriber_callback, ON_NEW_DATA);
}

void loop() {
    delay(100);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
```

5. Download the packages for your board type using the Boards Manager and select your specific board (and specify port).\
  5.1. Follow [this guide](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html) to download packages for ESP32 in Arduino IDE.
6. Verify and upload the sketch.
7. [Start and connect the Agent](../Agent.md).

### PlatformIO

NEW: Official instructions available in separate [PlatformIO repository](https://github.com/micro-ROS/micro_ros_platformio).

It's recommended to use PlatformIO together with [VSCode](https://platformio.org/install/ide?install=vscode).