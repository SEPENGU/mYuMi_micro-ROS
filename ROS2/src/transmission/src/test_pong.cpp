#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"

#define NODE_NAME "test_pong_node"
#define PILLAR_TOPIC "pillar"
#define STATUS_TOPIC "status"

using std::placeholders::_1;
using std::placeholders::_2;

class PongNode : public rclcpp::Node {
   public:
    PongNode(rclcpp::NodeOptions options) : Node(NODE_NAME, options) {
        // Initialize sensor data QoS profile
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

        // Initialize publishers and subscribers

        ping_ = this->create_subscription<std_msgs::msg::Header>("/mros/ping", qos,
                                                                 std::bind(&PongNode::ping_callback, this, _1));
        pong_ = this->create_publisher<std_msgs::msg::Header>("/ros2/pong", qos);
        peng_ = this->create_subscription<std_msgs::msg::Header>("/mros/peng", qos,
                                                                 std::bind(&PongNode::peng_callback, this, _1));

        pillar_ = this->create_publisher<std_msgs::msg::Float64>(PILLAR_TOPIC, 10);
        status_ = this->create_subscription<sensor_msgs::msg::JointState>(
            STATUS_TOPIC, 10, std::bind(&PongNode::status_callback, this, std::placeholders::_1));

        // Utilities

        nodeTime_ = this->get_clock();  // Create clock starting at the time of node creation
        posTime_ = nodeTime_->now();

        data_ = std::make_shared<std_msgs::msg::Header>();
        height_ = std::make_shared<sensor_msgs::msg::JointState>();
    }

   private:
    void ping_callback(const std_msgs::msg::Header::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Ping data: %s\n", msg->frame_id.data());
        data_ = msg;
        prevTime_ = nodeTime_->now();
        pong_->publish(*data_);

        if (adjust >= 0) {
            float currAdjust = 0.1 + std::stod(msg->frame_id) / 500;
            if (abs(adjust - currAdjust) >= 0.001) {
                adjust = currAdjust;
            }
        } else {
            adjust = 0.1 + std::stod(msg->frame_id) / 500;
        }

        auto range = std::make_shared<std_msgs::msg::Float64>();
        range->data = adjust;
        pillar_->publish(*range);
    }

    void peng_callback(const std_msgs::msg::Header::SharedPtr msg) {
        if (data_->stamp == msg->stamp) {
            currTime_ = nodeTime_->now();
            RCLCPP_INFO(this->get_logger(), "Peng data: %s\n", msg->frame_id.data());
            auto diff = currTime_.seconds() - prevTime_.seconds() +
                        (currTime_.nanoseconds() - prevTime_.nanoseconds()) / pow(10, 9);
            RCLCPP_INFO(this->get_logger(), "RTT: %fs\n", diff);
        } else {
            wrong++;
            RCLCPP_INFO(this->get_logger(), "Wrong timestamp(%d)\n", wrong);
        }
    }

    void status_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        height_ = msg;
        posTime_ = nodeTime_->now();
    }

    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr ping_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pong_;
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr peng_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pillar_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr status_;
    std_msgs::msg::Header::SharedPtr data_;
    sensor_msgs::msg::JointState::SharedPtr height_;
    rclcpp::Clock::SharedPtr nodeTime_;
    rclcpp::Time prevTime_;
    rclcpp::Time currTime_;
    rclcpp::Time posTime_;
    int wrong = 0;
    float adjust = -1.0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto pong_node = std::make_shared<PongNode>(rclcpp::NodeOptions());
    exec.add_node(pong_node);

    printf("spinning\n");
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
