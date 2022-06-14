#include <ledc.h>
#include <math.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/header.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <ultrasonic.h>
#include <unistd.h>
#include <uros_network_interfaces.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#endif

#if defined(CONFIG_MICRO_ROS_ESP_UART_TRANSPORT)
#include "esp32_serial_transport.h"
static size_t uart_port = UART_NUM_0;
#endif

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK)) {                                                   \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK)) {                                                     \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

#define STRING_BUFFER_LEN 50
#define MAX_DISTANCE_CM 500  // 5m max
#define TRIGGER_GPIO 17      // 17
#define ECHO_GPIO 16         // 16
#define BLINK 21
#define FREQ_MS 80

ultrasonic_sensor_t sensor = {.trigger_pin = TRIGGER_GPIO, .echo_pin = ECHO_GPIO};

rcl_publisher_t ping_pub;
rcl_subscription_t pong_sub;
rcl_publisher_t peng_pub;

std_msgs__msg__Header msg_header;

int state;
#ifdef CONFIG_PERCEPIO_TRACERECORDER_ENABLED
traceString chnState;
#endif

struct timespec ts;

void pwm_duty(int duty) {
    gpio_set_level(BLINK, state);
    // ledc_duty(state * DUTY_MAX);
}

void pwm_init() {
    gpio_reset_pin(BLINK);
    gpio_set_direction(BLINK, GPIO_MODE_OUTPUT);
    // ledc_init(BLINK);
}

void ping_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (state >= 1) {
        struct timespec currT;
        clock_gettime(CLOCK_REALTIME, &currT);
        if ((currT.tv_sec - ts.tv_sec) > 1) {
            state = 0;
            printf("State reset\n");
        }
    }
    if (timer != NULL && state < 1) {
        float distance;
        esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
        res = ESP_OK;
        if (res != ESP_OK) {
            printf("Error %d: ", res);
            switch (res) {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        } else {
            sprintf(msg_header.frame_id.data, "%f", distance * 100);
            msg_header.frame_id.size = strlen(msg_header.frame_id.data);

            // Fill the message timestamp
            clock_gettime(CLOCK_REALTIME, &ts);
            msg_header.stamp.sec = ts.tv_sec;
            msg_header.stamp.nanosec = ts.tv_nsec;

            printf("Ping data: %s\n", msg_header.frame_id.data);
            state = 1;
#ifdef CONFIG_PERCEPIO_TRACERECORDER_ENABLED
            vTracePrintF(chnState, "%d", state);
#endif
            pwm_duty(state);
            RCSOFTCHECK(rcl_publish(&ping_pub, (const void*)&msg_header, NULL));
        }
    }
}

void pong_sub_callback(const void* msgin) {
    struct timespec currT;
    clock_gettime(CLOCK_REALTIME, &currT);

    const std_msgs__msg__Header* msg = (const std_msgs__msg__Header*)msgin;
    if (msg_header.stamp.sec == msg->stamp.sec && msg_header.stamp.nanosec == msg->stamp.nanosec) {
        state = 0;
#ifdef CONFIG_PERCEPIO_TRACERECORDER_ENABLED
        vTracePrintF(chnState, "%d", state);
#endif
        pwm_duty(state);

        float RTT = currT.tv_sec - ts.tv_sec + (currT.tv_nsec - ts.tv_nsec) / pow(10, 9);
        printf("Pong data: %s, RTT: %fs\r\n", msg->frame_id.data, RTT);

        RCSOFTCHECK(rcl_publish(&peng_pub, (const void*)&msg_header, NULL));
    } else {
        state = 0.5;
#ifdef CONFIG_PERCEPIO_TRACERECORDER_ENABLED
        vTracePrintF(chnState, "%d", 0);
#endif
        pwm_duty(0);
        printf("Wrong timestamp\r\n");
    }
}

void low_prio_task(void* arg) {
    while (1) {
        for (int i = 0; i < 3000; i++)
            ;
        vTaskDelay(777 / portTICK_PERIOD_MS);
    }
}

void idle_task(void* arg) {
    while (1) {
        vTaskDelay(1234 / portTICK_PERIOD_MS);
    }
}

void micro_ros_task(void* arg) {
#ifdef CONFIG_PERCEPIO_TRACERECORDER_ENABLED
    chnState = xTraceRegisterString("state");
#endif
    printf("Init\r\n");
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Static Agent IP and port can be used instead of autodiscovery.
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    // RCCHECK(rmw_uros_options_set_udp_address("192.168.0.111", "8888", rmw_options));  // Manual
#endif
#else
    while (RMW_RET_OK != rmw_uros_ping_agent(1000, 1))
#endif

    // create init_options
    size_t domain_id = (size_t)50;
    RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    vTaskDelay(pdMS_TO_TICKS(1000));
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "ping_esp32_node", "", &support));

    vTaskDelay(pdMS_TO_TICKS(1000));
    const rosidl_message_type_support_t* type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header);
    RCCHECK(rclc_publisher_init_best_effort(&ping_pub, &node, type_support, "/mros/ping"));
    vTaskDelay(pdMS_TO_TICKS(1000));
    RCCHECK(rclc_subscription_init_best_effort(&pong_sub, &node, type_support, "/ros2/pong"));
    vTaskDelay(pdMS_TO_TICKS(1000));
    RCCHECK(rclc_publisher_init_best_effort(&peng_pub, &node, type_support, "/mros/peng"));

    // Create a 3 seconds ping timer,
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(FREQ_MS), ping_timer_callback));

    // Create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &pong_sub, &msg_header, &pong_sub_callback, ON_NEW_DATA));

    // Create and allocate the pingpong messages
    msg_header.frame_id.data = (char*)malloc(sizeof(char) * STRING_BUFFER_LEN);
    msg_header.frame_id.size = 0;
    msg_header.frame_id.capacity = STRING_BUFFER_LEN;

    ultrasonic_init(&sensor);
    pwm_init();

    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(FREQ_MS / 2));
        usleep(10000);
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&ping_pub, &node));
    RCCHECK(rcl_subscription_fini(&pong_sub, &node));
    RCCHECK(rcl_publisher_fini(&peng_pub, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void app_main(void) {
    clock_gettime(CLOCK_REALTIME, &ts);
    printf("Init\r\n");
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
//#elif defined(RMW_UXRCE_TRANSPORT_CUSTOM)
#elif defined(CONFIG_MICRO_ROS_ESP_UART_TRANSPORT)
    rmw_uros_set_custom_transport(true, (void*)&uart_port, esp32_serial_open, esp32_serial_close, esp32_serial_write,
                                  esp32_serial_read);
#else
#error micro-ROS transports misconfigured
#endif

    // pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
    xTaskCreate(low_prio_task, "running_task", CONFIG_MICRO_ROS_APP_STACK, NULL, 1, NULL);
    xTaskCreate(idle_task, "idle_task", 1000, NULL, 0, NULL);
}
