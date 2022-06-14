#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/header.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STRING_BUFFER_LEN 50
#define BLINK 7
#define DUTY_MAX 255
#define FREQ_MS 100

#define RCCHECK(fn)                    \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
            error_loop();              \
        }                              \
    }
#define RCSOFTCHECK(fn)                \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
        }                              \
    }

rcl_publisher_t ping_pub;
rcl_subscription_t pong_sub;
rcl_publisher_t peng_pub;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

std_msgs__msg__Header msg_header;

int state = 0;
const int trigPin = 2;
const int echoPin = 3;
float duration, distance;

struct dayTime {
    int tv_sec;
    int tv_usec;
};
struct dayTime ts;

void gettimeofday(dayTime* timeStruct, void* ignore) {
    timeStruct->tv_sec = millis() / 1000;
    timeStruct->tv_usec = micros();
}

void error_loop() {
    while (1) {
        // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    }
}

void ping_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (state >= 1) {
        struct dayTime currT;
        gettimeofday(&currT, NULL);
        if ((currT.tv_sec - ts.tv_sec) > 1) {
            state = 0;
            printf("State reset\r\n");
        }
    }
    if (timer != NULL && state < 1) {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        duration = pulseIn(echoPin, HIGH);
        distance = (duration * .0343) / 2;

        sprintf(msg_header.frame_id.data, "%f", distance);
        msg_header.frame_id.size = strlen(msg_header.frame_id.data);

        // Fill the message timestamp
        gettimeofday(&ts, NULL);
        msg_header.stamp.sec = ts.tv_sec;
        msg_header.stamp.nanosec = ts.tv_usec;

        printf("Ping data: %s\r\n", msg_header.frame_id.data);
        state = 1;
        // digitalWrite(BLINK, state);
        analogWrite(BLINK, state * DUTY_MAX);
        rcl_publish(&ping_pub, (const void*)&msg_header, NULL);
    }
}

void pong_sub_callback(const void* msgin) {
    struct dayTime currT;
    gettimeofday(&currT, NULL);

    const std_msgs__msg__Header* msg = (const std_msgs__msg__Header*)msgin;
    if (msg_header.stamp.sec == msg->stamp.sec && msg_header.stamp.nanosec == msg->stamp.nanosec) {
        state = 0;
        // digitalWrite(BLINK, state);
        analogWrite(BLINK, state * DUTY_MAX);

        float RT_T = currT.tv_sec - ts.tv_sec + (currT.tv_usec - ts.tv_usec) / pow(10, 6);
        printf("Pong data: %s, RTT: %fs\r\n", msg->frame_id.data, RT_T);

        RCSOFTCHECK(rcl_publish(&peng_pub, (const void*)&msg_header, NULL));
    } else {
        state = 0.5;
        // digitalWrite(BLINK, LOW);
        analogWrite(BLINK, state * DUTY_MAX);
    }
}

void setup() {
    gettimeofday(&ts, NULL);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(BLINK, OUTPUT);

    set_microros_transports();

    while (RMW_RET_OK != rmw_uros_ping_agent(1000, 1))
        ;

    allocator = rcl_get_default_allocator();

    // create init_options
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    size_t domain_id = (size_t)50;
    rcl_init_options_set_domain_id(&init_options, domain_id);
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "ping_due_node", "", &support));

    const rosidl_message_type_support_t* type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header);
    RCCHECK(rclc_publisher_init_best_effort(&ping_pub, &node, type_support, "/mros/ping"));
    RCCHECK(rclc_subscription_init_best_effort(&pong_sub, &node, type_support, "/ros2/pong"));
    RCCHECK(rclc_publisher_init_best_effort(&peng_pub, &node, type_support, "/mros/peng"));

    // Create a 3 seconds ping timer,
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(FREQ_MS), ping_timer_callback));

    // Create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &pong_sub, &msg_header, &pong_sub_callback, ON_NEW_DATA));

    // Create and allocate the pingpong messages
    msg_header.frame_id.data = (char*)malloc(sizeof(char) * STRING_BUFFER_LEN);
    msg_header.frame_id.size = 0;
    msg_header.frame_id.capacity = STRING_BUFFER_LEN;
}

void loop() {
    delay(100);
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(FREQ_MS / 2)));
}
