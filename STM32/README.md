# micro-ROS for STM32Cube

This guide sets up micro-ROS on STM32CubeIDE, based on the [STM32Cube guide](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/blob/galactic/README.md) by micro-ROS.

Tested with NUCLEO-F767ZI and STM32F3DISCOVERY.

## Setup

1. Download and install [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).
2. Create a new CubeIDE project for your specific board.

      2.1 If NUCLEO-F767ZI is used, one of the projects in this folder (STM32) can be imported and only step 3 needs to be followed for micro-ROS to work.

3. Clone the Galactic version of the [micro-ROS utilities](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/tree/galactic) at the root of the selected project.

4. Configure the following project settings:
      
      4.1. Go to `Project -> Settings -> C/C++ Build -> Settings -> Build Steps Tab` and in `Pre-build steps` add:

    ```bash
    docker pull microros/micro_ros_static_library_builder:galactic && docker run --rm -v ${workspace_loc:/${ProjName}}:/project --env 
    MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:galactic
    ```

      4.2. Add the micro-ROS include directory to the project:\
      In `Project -> Settings -> C/C++ Build -> Settings -> Tool Settings Tab -> MCU GCC Compiler -> Include paths` add 
      `../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include`.

      4.3. Add the micro-ROS precompiled library to the project:\
      In `Project -> Settings -> C/C++ Build -> Settings -> MCU GCC Linker -> Libraries`,
      - add `<ABSOLUTE_PATH_TO>/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros` in `Library search path (-L)`,
      - add `microros` in `Libraries (-l)`.

5. Add the following source code files from `extra_sources/` to `Core/Src/`:
      - `microros_time.c`
      - `microros_allocators.c`
      - `custom_memory_manager.c`
      - From `microros_transports/`, add either `dma_transport.c` for USART with DMA (recommended), or `it_transport.c` for USART with interrupts.

6. Create a FreeRTOS task with enough stack size for micro-ROS (at least 200 words).
7. Configure USART as the transport layer:

      6.1. Enable USART in your STM32CubeMX. If you plan on using UART via the built-in USB port, make sure you're selecting the correct USART (check the user manual for your board).

      6.2. For the selected USART, enable `global interrupt` under `NVIC Settings`.

      6.3. USART with DMA (recommended, skip this step for USART with interrupts):
      - For the selected USART, enable DMA for Tx and Rx under `DMA Settings`
      - Set the DMA priotity to `Very High` for Tx and Rx
      - Set the DMA mode to `Circular` for Rx

8. To start with micro-ROS in STM32Cube, look at either [`sample_main.c`](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/blob/galactic/sample_main.c) in the utilities, one of the projects in this folder (STM32), or the following example:

```C
.
.
.
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// Standard libraries

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>  // Example message type for topics

/* USER CODE END Includes */
.
.
.
/* USER CODE BEGIN 4 */

void publisher_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    msg_pub.data++;
    rcl_publish(&publisher, &msg_pub, NULL);
  }
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  printf("Received: %d", msg->data);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */

// Transport initialisation
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

// micro-ROS memory allocation

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

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

/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  rmw_uros_set_custom_transport(
    true,
    (void *) &huart3,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Error on default allocators (line %d)\n", __LINE__); 
  }

  // micro-ROS app

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "mros_cubemx_node", "", &support);

  const rosidl_message_type_support_t* type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  rclc_publisher_init_default(&publisher, &node, type_support, "mros_pub");
  rclc_subscription_init_default(&subscriber, &node, type_support, "mros_sub");
  
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), publisher_callback);
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscriber_callback, ON_NEW_DATA);

  for(;;)
  {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    osDelay(100);
  }

  // free resources
  rcl_publisher_fini(&publisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rcl_node_fini(&node);

  vTaskDelete(NULL);

  /* USER CODE END 5 */
}
.
.
.
```

6. Build and run the project.
7. Start and connect the [Agent](../Agent.md).

### Troubleshoot

TODO