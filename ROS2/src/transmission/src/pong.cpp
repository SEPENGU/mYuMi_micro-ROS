#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <tuple>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"

#define NODE_NAME "pong_node"
#define PILLAR_TOPIC "pillar"
#define STATUS_TOPIC "status"
#define THRESHOLD 1   // Precision to set height of pillar
#define UPPER 0.9     // Max allowed height to set pillar
#define LOWER 0.01    // Min allowed height to set pillar
#define MAX_DIST 300  // Max distance of rangefinder

using std::placeholders::_1;
using std::placeholders::_2;

class PongNode : public rclcpp::Node {
   public:
    PongNode(rclcpp::NodeOptions options) : Node(NODE_NAME, options) {
        callback_group_subscribers_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_service_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto subs_opt = rclcpp::SubscriptionOptions();
        subs_opt.callback_group = callback_group_subscribers_;

        // Initialize sensor data QoS profile
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

        // Initialize publishers and subscribers

        ping_ = this->create_subscription<std_msgs::msg::Header>(
            "/mros/ping", qos, std::bind(&PongNode::ping_callback, this, _1), subs_opt);
        pong_ = this->create_publisher<std_msgs::msg::Header>("/ros2/pong", qos);
        peng_ = this->create_subscription<std_msgs::msg::Header>(
            "/mros/peng", qos, std::bind(&PongNode::peng_callback, this, _1), subs_opt);

        status_ = this->create_subscription<sensor_msgs::msg::JointState>(
            STATUS_TOPIC, 10, std::bind(&PongNode::status_callback, this, std::placeholders::_1), subs_opt);

        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "/ros2/PongNode", std::bind(&PongNode::set_switch, this, _1, _2), ::rmw_qos_profile_default,
            callback_group_service_);
        pillar_ = this->create_publisher<std_msgs::msg::Float64>(PILLAR_TOPIC, 10);

        // Utilities

        nodeTime_ = this->get_clock();  // Create clock starting at the time of node creation
        posTime_ = nodeTime_->now();

        data_ = std::make_shared<std_msgs::msg::Header>();
        joints_ = std::make_shared<sensor_msgs::msg::JointState>();
        height_ = std::make_shared<std_msgs::msg::Float64>();
    }

   private:
    void ping_callback(const std_msgs::msg::Header::SharedPtr msg) {
        // TODO: If mYuMi is still responding
        currTime_ = nodeTime_->now();
        if ((currTime_ - posTime_).seconds() > 10) {
        }

        RCLCPP_INFO(this->get_logger(), "Ping data: %s\n", msg->frame_id.data());
        data_ = msg;

        prevTime_ = nodeTime_->now();
        pong_->publish(*data_);

        if (set == 1) pillar_->publish(*height_);
    }

    void peng_callback(const std_msgs::msg::Header::SharedPtr msg) {
        if (data_->stamp == msg->stamp) {
            currTime_ = nodeTime_->now();
            // RCLCPP_INFO(this->get_logger(), "Peng data: %s\n", msg->frame_id.data());
            auto diff = currTime_.seconds() - prevTime_.seconds() +
                        (currTime_.nanoseconds() - prevTime_.nanoseconds()) / pow(10, 9);
            // RCLCPP_INFO(this->get_logger(), "RTT: %fs\n", diff);
        } else {
            wrong++;
            RCLCPP_INFO(this->get_logger(), "Wrong timestamp(%d)\n", wrong);
        }
    }

    void status_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        joints_ = msg;
        posTime_ = nodeTime_->now();
        read = 1;
    }

    void set_switch(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
        rclcpp::sleep_for(std::chrono::seconds(1));  // Wait for arm to adjust
        set = request->a;
        int req_dist = request->b;  // Requested distance of 1D rangefinder

        if (set == 1 && read == 1) {
            float curr_dist = std::stof(data_->frame_id);  // Current distance of 1D rangefinder
            float curr_height = joints_->position[8];      // Current height of pillar
            float difference = abs(req_dist - curr_dist);
            while (difference > THRESHOLD) {
                float adjust = 2 * difference * (UPPER - LOWER) / MAX_DIST;  // Adjust based on difference
                float new_height = 0;
                if (req_dist - curr_dist >= 0)
                    new_height = curr_height + adjust;
                else
                    new_height = curr_height - adjust;

                if (new_height < LOWER)
                    new_height = LOWER;
                else if (new_height > UPPER)
                    new_height = UPPER;

                RCLCPP_INFO(this->get_logger(), "Height difference: %f, adjusting by: %f\n", difference, adjust);
                // RCLCPP_INFO(this->get_logger(), "curr_height: %f, new_height: %f\n", curr_height, new_height);
                height_->data = new_height;
                rclcpp::sleep_for(std::chrono::seconds(1));

                curr_dist = std::stof(data_->frame_id);
                curr_height = joints_->position[8];
                difference = abs(req_dist - curr_dist);
            }
            // After correctly adjusting
            set = 0;
            response->sum = 1;
            RCLCPP_INFO(this->get_logger(), "Adjustment finished\n");
        } else if (read == 0) {
            RCLCPP_INFO(this->get_logger(), "Cannot read joint states\n");
        } else if (set == 0) {
            response->sum = 0;
            RCLCPP_INFO(this->get_logger(), "Adjustment deactivated\n");
        } else {
            response->sum = -1;
            RCLCPP_INFO(this->get_logger(), "Number request out of range\n");
        }
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_;
    rclcpp::CallbackGroup::SharedPtr callback_group_service_;
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr ping_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pong_;
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr peng_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr status_;
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pillar_;
    std_msgs::msg::Header::SharedPtr data_;
    sensor_msgs::msg::JointState::SharedPtr joints_;
    std_msgs::msg::Float64::SharedPtr height_;
    rclcpp::Clock::SharedPtr nodeTime_;
    rclcpp::Time prevTime_;
    rclcpp::Time currTime_;
    rclcpp::Time posTime_;
    int wrong = 0;  // Number of wrong timestamps
    int read = 0;   // If status has been read
    int set = 0;    // Activate system
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;
    auto topic_node = std::make_shared<PongNode>(rclcpp::NodeOptions());

    exec.add_node(topic_node);

    printf("spinning\n");
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
