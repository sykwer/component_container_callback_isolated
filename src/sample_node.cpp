#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"

// StaticCallbackDedicatedMultiThreadedExecutor - from here
class StaticCallbackDedicatedMultiThreadedExecutor : public rclcpp::Executor {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(StaticCallbackDedicatedMultiThreadedExecutor)

  RCLCPP_PUBLIC
  explicit StaticCallbackDedicatedMultiThreadedExecutor(
      const rclcpp::ExecutorOptions &options = rclcpp::ExecutorOptions());

  RCLCPP_PUBLIC
  virtual ~StaticCallbackDedicatedMultiThreadedExecutor();

  RCLCPP_PUBLIC
  void spin() override;

  RCLCPP_PUBLIC
  void add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node, bool notify = true) override;

  RCLCPP_PUBLIC
  void add_node(std::shared_ptr<rclcpp::Node> node, bool notify = true) override;
};

StaticCallbackDedicatedMultiThreadedExecutor::StaticCallbackDedicatedMultiThreadedExecutor(
    const rclcpp::ExecutorOptions &options) : rclcpp::Executor(options) {
}

StaticCallbackDedicatedMultiThreadedExecutor::~StaticCallbackDedicatedMultiThreadedExecutor() {
}

void StaticCallbackDedicatedMultiThreadedExecutor::spin() {
}

void StaticCallbackDedicatedMultiThreadedExecutor::add_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node, bool notify) {
  (void) node;
  (void) notify;
}

void StaticCallbackDedicatedMultiThreadedExecutor::add_node(
    std::shared_ptr<rclcpp::Node> node, bool notify) {
  this->add_node(node->get_node_base_interface(), notify);
}

// StaticCallbackDedicatedMultiThreadedExecutor - to here

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

  auto node = std::make_shared<SampleNode>();
  auto executor = std::make_shared<StaticCallbackDedicatedMultiThreadedExecutor>();

  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}

