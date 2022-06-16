# micro-ROS for ESP-IDF

This guide sets up micro-ROS with ESP-IDF v4.3, based on the [ESP-IDF guide](https://github.com/micro-ROS/micro_ros_espidf_component/blob/galactic/README.md) by micro-ROS.

Tested with ESP32-WROOM-32E, Adafruit HUZZAH32, and NodeMCU-32S.

## Setup

WARNING: Do not download the ESP-IDF prerequisites, this guide will not work if you do.

1. Clone [ESP-IDF v4.3](https://github.com/espressif/esp-idf/tree/release/v4.3) and run `install.sh` to install dependencies.\
    1.1. (Optional) Create an alias to run `export.sh`.
2. Export the environment variables by running `export.sh` inside ESP-IDF in a terminal, and then install packages inside the environment for micro-ROS with:
    ```bash
    pip3 install catkin_pkg lark-parser empy colcon-common-extensions importlib-resources
    ```
3. To start with a micro-ROS ESP-IDF project, either use one of the [examples](https://github.com/micro-ROS/micro_ros_espidf_component/tree/galactic/examples) included in the component, use one of the projects in this folder (ESP32), or use the following example code for publishers and subscribers, in a [ESP-IDF project](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html):

```C
#include <unistd.h>

// ESP-IDF libraries

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Standard micro-ROS libraries

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h> // Example message type for topics
#include <uros_network_interfaces.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#endif

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

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	rcl_init_options_init(&init_options, allocator);

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Wi-Fi
	rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);
#endif

	rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

	rcl_node_t node;
	rclc_node_init_default(&node, "mros_esp32_node", "", &support);

	const rosidl_message_type_support_t* type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
    rclc_publisher_init_default(&publisher, &node, type_support, "mros_pub");
    rclc_subscription_init_default(&subscriber, &node, type_support, "mros_sub");

    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), publisher_callback);

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscriber_callback, ON_NEW_DATA);

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	rcl_publisher_fini(&publisher, &node);
    rcl_subscription_fini(&subscriber, &node);
	rcl_node_fini(&node);

  	vTaskDelete(NULL);
}

void app_main(void) {
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    xTaskCreate(micro_ros_task,
            "uros_task",
            16000,
            NULL,
            5,
            NULL);
}
```

4. Create a folder named "components" at the root of the project, and inside it, clone the Galactic version of the [micro-ROS component](https://github.com/micro-ROS/micro_ros_espidf_component/tree/galactic).
5. Put a terminal at the root of the project and run the following commands:

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

    5.1. Instead of using the terminal to build and flash the project, it can be done inside [Eclipse](https://www.eclipse.org/downloads/packages/release/2021-06/r/eclipse-ide-cc-developers) by using the [ESP-IDF Eclipse Plugin](https://github.com/espressif/idf-eclipse-plugin).

6. Start and connect the [Agent](../Agent.md).

### Troubleshoot

* If the build stops near the end on something like `libmicroros.a`, just let it be for half an hour or so. It takes time when you first build it since it needs to download packages and statically compile them.
* If you encounter issues during the build process, ensure that you are running in a clean shell environment without the ROS 2 setup script sourced.
* If the first build after cloning the component fails, the packages in the component won't fully download and won't resume when rebuilding, so you need to either run `idf.py clean-microros` inside the component or delete the component and clone it again.
* If you change between serial and UDP you need to either run `idf.py clean-microros` inside the component or delete the component and clone it again, as the component only downloads the necessary packages during the initial build.
* If UDP is used and it keeps failing on `rclc_support_init_with_options()`, make sure you specified the correct IP and port in `menuconfig` and that both the host PC and the ESP32 is reachable by pinging. If it still doesn't work, hard code the IP and port in `rmw_uros_options_set_udp_address()`.
* The serial monitor will only work properly with UDP since micro-ROS uses the UART via USB.