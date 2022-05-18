#include <Arduino.h>
#include <Servo.h>
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/header.h>
#include <stdio.h>

#define STRING_BUFFER_LEN 50
#define BLINK 7
#define DUTY_MAX 255
#define SERVO_PIN 9
#define MIN_PULSE_WIDTH 800
#define MAX_PULSE_WIDTH 2200
#define DEFAULT_PULSE_WIDTH 1500
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

rcl_subscription_t ping_sub;
rcl_publisher_t pong_pub;
rcl_subscription_t peng_sub;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

std_msgs__msg__Header msg_header;

int state = 0;
Servo myservo;

struct dayTime {
    int tv_sec;
    int tv_usec;
};
struct dayTime ts;

void gettimeofday(dayTime *timeStruct, void *ignore);
void gettimeofday(dayTime *timeStruct, void *ignore) {
    timeStruct->tv_sec = millis() / 1000;
    timeStruct->tv_usec = micros();
}

void error_loop() {
    while (1) {
        delay(100);
    }
}

void ping_sub_callback(const void *msgin) {
    const std_msgs__msg__Header *msg = (const std_msgs__msg__Header *)msgin;

    myservo.write(atoi(msg->frame_id.data) + 90 - 15);  // Only works between [-75, 75]

    sprintf(msg_header.frame_id.data, "%s", msg->frame_id.data);
    msg_header.frame_id.size = strlen(msg_header.frame_id.data);

    gettimeofday(&ts, NULL);
    msg_header.stamp.sec = msg->stamp.sec;
    msg_header.stamp.nanosec = msg->stamp.nanosec;

    state = 1;
    // digitalWrite(BLINK, state);
    analogWrite(BLINK, state * DUTY_MAX);
    RCCHECK(rcl_publish(&pong_pub, (const void *)&msg_header, NULL));
}

void peng_sub_callback(const void *msgin) {
    struct dayTime currT;
    gettimeofday(&currT, NULL);

    const std_msgs__msg__Header *msg = (const std_msgs__msg__Header *)msgin;
    if (msg_header.stamp.sec == msg->stamp.sec && msg_header.stamp.nanosec == msg->stamp.nanosec) {
        state = 0;
        // digitalWrite(BLINK, state);
        analogWrite(BLINK, state * DUTY_MAX);

        float RT_T = currT.tv_sec - ts.tv_sec + (currT.tv_usec - ts.tv_usec) / pow(10, 6);
        printf("RTT: %f", RT_T);
    } else {
        state = 0.5;
        // digitalWrite(BLINK, LOW);
        analogWrite(BLINK, state * DUTY_MAX);
    }
}

void setup() {
    myservo.attach(SERVO_PIN);
    gettimeofday(&ts, NULL);
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
    RCCHECK(rclc_node_init_default(&node, "pong_due_node", "", &support));

    const rosidl_message_type_support_t *type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header);
    RCCHECK(rclc_subscription_init_best_effort(&ping_sub, &node, type_support, "/ros2/ping"));
    RCCHECK(rclc_publisher_init_best_effort(&pong_pub, &node, type_support, "/mros/pong"));
    RCCHECK(rclc_subscription_init_best_effort(&peng_sub, &node, type_support, "/ros2/peng"));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &ping_sub, &msg_header, &ping_sub_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &peng_sub, &msg_header, &peng_sub_callback, ON_NEW_DATA));

    msg_header.frame_id.data = (char *)malloc(sizeof(char) * STRING_BUFFER_LEN);
    msg_header.frame_id.size = 0;
    msg_header.frame_id.capacity = STRING_BUFFER_LEN;
}

void loop() {
    delay(100);
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(FREQ_MS / 2)));
}
