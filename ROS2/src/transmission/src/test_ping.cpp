#include <math.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

#define NODE_NAME "test_ping_node"
#define STATUS_TOPIC "status"

using std::placeholders::_1;
using std::placeholders::_2;

class PingNode : public rclcpp::Node {
   public:
    PingNode(rclcpp::NodeOptions options) : Node(NODE_NAME, options) {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

        this->declare_parameter<std::int32_t>("my_ms", 100);
        this->get_parameter("my_ms", param_ms_);

        ping_ = this->create_publisher<std_msgs::msg::Header>("/ros2/ping", qos);
        timer_ =
            this->create_wall_timer(std::chrono::milliseconds(param_ms_), std::bind(&PingNode::ping_callback, this));
        pong_ = this->create_subscription<std_msgs::msg::Header>("/mros/pong", qos,
                                                                 std::bind(&PingNode::pong_callback, this, _1));
        peng_ = this->create_publisher<std_msgs::msg::Header>("/ros2/peng", qos);

        status_ = this->create_subscription<sensor_msgs::msg::JointState>(
            STATUS_TOPIC, 10, std::bind(&PingNode::status_callback, this, _1));

        nodeTime_ = this->get_clock();  // Create clock starting at the time of node creation
        prevTime_ = nodeTime_->now();
        posTime_ = nodeTime_->now();

        data_ = std::make_shared<std_msgs::msg::Header>();
        height_ = std::make_shared<sensor_msgs::msg::JointState>();
    }

   private:
    void ping_callback() {
        if (state == 1) {
            currTime_ = nodeTime_->now();
            if ((currTime_.seconds() - prevTime_.seconds()) > 1) state = 0;
        } else if (read == 1) {
            int angle = (int)round(height_->position[8] * 90);
            data_->frame_id = std::to_string(angle);
            RCLCPP_INFO(this->get_logger(), "Ping data: %s\n", data_->frame_id.data());

            prevTime_ = nodeTime_->now();
            data_->stamp.sec = prevTime_.seconds();
            data_->stamp.nanosec = prevTime_.nanoseconds();

            ping_->publish(*data_);
            state = 1;
        }
    }

    void pong_callback(const std_msgs::msg::Header::SharedPtr msg) {
        if (data_->stamp == msg->stamp) {
            currTime_ = nodeTime_->now();
            state = 0;
            RCLCPP_INFO(this->get_logger(), "Pong data: %s\n", msg->frame_id.data());

            auto diff = currTime_.seconds() - prevTime_.seconds() +
                        (currTime_.nanoseconds() - prevTime_.nanoseconds()) / pow(10, 9);
            RCLCPP_INFO(this->get_logger(), "RTT: %fs\n", diff);

            data_ = msg;
            peng_->publish(*data_);
        } else {
            wrong++;
            RCLCPP_INFO(this->get_logger(), "Wrong timestamp(%d)\n", wrong);
        }
    }

    void status_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        height_ = msg;
        posTime_ = nodeTime_->now();
        read = 1;
    }

    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr ping_;
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr pong_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr peng_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr status_;
    std_msgs::msg::Header::SharedPtr data_;
    sensor_msgs::msg::JointState::SharedPtr height_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock::SharedPtr nodeTime_;
    rclcpp::Time prevTime_;
    rclcpp::Time currTime_;
    rclcpp::Time posTime_;
    std::int32_t param_ms_;
    int state = 0;
    int wrong = 0;
    int read = 0;  // If status has been read
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto ping_node = std::make_shared<PingNode>(rclcpp::NodeOptions());
    exec.add_node(ping_node);

    printf("spinning\n");
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
