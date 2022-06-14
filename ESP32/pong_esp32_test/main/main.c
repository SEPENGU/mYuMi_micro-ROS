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
#include <unistd.h>
#include <uros_network_interfaces.h>

#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "driver/uart.h"
#include "esp32_serial_transport.h"
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
#define BLINK 21

#define SERVO_MIN_PULSEWIDTH_US (800)   // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US (2200)  // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE (90)           // Maximum angle in degree upto which servo can rotate
#define SERVO_PULSE_GPIO (22)           // GPIO connects to the PWM signal line
#define MS_DELAY (10)                   // Callback delay
#define FREQ_MS 80

rcl_subscription_t ping_sub;
rcl_publisher_t pong_pub;
rcl_subscription_t peng_sub;

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

void ping_sub_callback(const void* msgin) {
    const std_msgs__msg__Header* msg = (const std_msgs__msg__Header*)msgin;

    int32_t angle = atoi(msg->frame_id.data);
    printf("Angle: %d\n", angle);
    uint32_t duty_us =
        (angle + SERVO_MAX_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (2 * SERVO_MAX_DEGREE) +
        SERVO_MIN_PULSEWIDTH_US;
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_us));
    vTaskDelay(pdMS_TO_TICKS(MS_DELAY));

    sprintf(msg_header.frame_id.data, "%s", msg->frame_id.data);
    msg_header.frame_id.size = strlen(msg_header.frame_id.data);

    clock_gettime(CLOCK_REALTIME, &ts);
    msg_header.stamp.sec = msg->stamp.sec;
    msg_header.stamp.nanosec = msg->stamp.nanosec;

    printf("Ping data: %s\n", msg->frame_id.data);
    state = 1;
#ifdef CONFIG_PERCEPIO_TRACERECORDER_ENABLED
    vTracePrintF(chnState, "%d", state);
#endif
    pwm_duty(state);
    RCCHECK(rcl_publish(&pong_pub, (const void*)&msg_header, NULL));
}

void peng_sub_callback(const void* msgin) {
    struct timespec currT;
    clock_gettime(CLOCK_REALTIME, &currT);

    const std_msgs__msg__Header* msg = (const std_msgs__msg__Header*)msgin;

    printf("hsec: %d, msec: %d, is: %d\r\n", msg_header.stamp.nanosec, msg->stamp.nanosec,
           msg_header.stamp.nanosec == msg->stamp.nanosec);
    if (msg_header.stamp.sec == msg->stamp.sec && msg_header.stamp.nanosec == msg->stamp.nanosec) {
        state = 0;
#ifdef CONFIG_PERCEPIO_TRACERECORDER_ENABLED
        vTracePrintF(chnState, "%d", state);
#endif
        pwm_duty(state);

        float RTT = currT.tv_sec - ts.tv_sec + (currT.tv_nsec - ts.tv_nsec) / pow(10, 9);
        printf("Pong data: %s, RTT: %fs\r\n", msg->frame_id.data, RTT);
    } else {
        state = 0.5;
#ifdef CONFIG_PERCEPIO_TRACERECORDER_ENABLED
        vTracePrintF(chnState, "%d", 0);
#endif
        pwm_duty(state);
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

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Static Agent IP and port can be used instead of autodiscovery.
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    // RCCHECK(rmw_uros_options_set_udp_address("192.168.0.111", "8888", rmw_options)); // Manual
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
    RCCHECK(rclc_node_init_default(&node, "pong_esp32_node", "", &support));

    vTaskDelay(pdMS_TO_TICKS(1000));
    const rosidl_message_type_support_t* type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header);
    RCCHECK(rclc_subscription_init_best_effort(&ping_sub, &node, type_support, "/ros2/ping"));
    vTaskDelay(pdMS_TO_TICKS(1000));
    RCCHECK(rclc_publisher_init_best_effort(&pong_pub, &node, type_support, "/mros/pong"));
    vTaskDelay(pdMS_TO_TICKS(1000));
    RCCHECK(rclc_subscription_init_best_effort(&peng_sub, &node, type_support, "/ros2/peng"));

    // Create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &ping_sub, &msg_header, &ping_sub_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &peng_sub, &msg_header, &peng_sub_callback, ON_NEW_DATA));

    // Create and allocate the pingpong messages
    msg_header.frame_id.data = (char*)malloc(sizeof(char) * STRING_BUFFER_LEN);
    msg_header.frame_id.size = 0;
    msg_header.frame_id.capacity = STRING_BUFFER_LEN;

    pwm_init();

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PULSE_GPIO);
    mcpwm_config_t pwm_config = {
        .frequency = 50,  // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,      // duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(FREQ_MS / 2));
        usleep(10000);
    }

    // free resources
    RCCHECK(rcl_subscription_fini(&ping_sub, &node));
    RCCHECK(rcl_publisher_fini(&pong_pub, &node));
    RCCHECK(rcl_subscription_fini(&peng_sub, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void app_main(void) {
    clock_gettime(CLOCK_REALTIME, &ts);

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
    // vTaskDelay(10000 / portTICK_PERIOD_MS);
    xTaskCreate(low_prio_task, "running_task", CONFIG_MICRO_ROS_APP_STACK, NULL, 1, NULL);
    xTaskCreate(idle_task, "idle_task", 1000, NULL, 0, NULL);
}
