#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/dynamic_size_array.hpp"

using namespace std::chrono_literals;

const long long MESSAGE_SIZE = 1024;

class SampleNode : public rclcpp::Node {
public:
  SampleNode() : Node("sample_node"), count_(0) {
    publisher_ = this->create_publisher<interfaces::msg::DynamicSizeArray>("topic_out", 1);

    timer_ = this->create_wall_timer(3000ms, std::bind(&SampleNode::timer_callback, this));

    subscription_ = this->create_subscription<interfaces::msg::DynamicSizeArray>(
      "topic_in", 1, std::bind(&SampleNode::subscription_callback, this, std::placeholders::_1));
  }

private:
  void timer_callback() {
    auto message = interfaces::msg::DynamicSizeArray();
    message.id = count_++;
    message.data.resize(MESSAGE_SIZE);
    RCLCPP_INFO(this->get_logger(), "Publishing Message ID: '%ld'", message.id);
    publisher_->publish(std::move(message));
  }

  void subscription_callback(const interfaces::msg::DynamicSizeArray::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard message ID: '%ld'", msg->id);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<interfaces::msg::DynamicSizeArray>::SharedPtr publisher_;
  rclcpp::Subscription<interfaces::msg::DynamicSizeArray>::SharedPtr subscription_;
  size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SampleNode>());
  rclcpp::shutdown();
  return 0;
}

