#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <sys/syscall.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "interfaces/msg/dynamic_size_array.hpp"

using namespace std::chrono_literals;

const long long MESSAGE_SIZE = 1024;

class SampleNode : public rclcpp::Node {
public:
  explicit SampleNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("sample_node", options), count_(0) {
    publisher_ = this->create_publisher<interfaces::msg::DynamicSizeArray>("topic_out", 1);

    group1_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    group2_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    group3_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    timer_ = this->create_wall_timer(3000ms, std::bind(&SampleNode::timer_callback, this), group1_);
    timer2_ = this->create_wall_timer(1333ms, std::bind(&SampleNode::timer_callback2, this), group2_);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = group3_;

    subscription_ = this->create_subscription<interfaces::msg::DynamicSizeArray>(
      "topic_in", 1, std::bind(&SampleNode::subscription_callback, this, std::placeholders::_1), sub_options);
  }

private:
  void timer_callback() {
    long tid = syscall(SYS_gettid);
    auto message = interfaces::msg::DynamicSizeArray();
    message.id = count_++;
    message.data.resize(MESSAGE_SIZE);
    RCLCPP_INFO(this->get_logger(), "Publishing Message ID (timer_callback tid=%ld): '%ld'", tid, message.id);
    publisher_->publish(std::move(message));
  }

  void timer_callback2() {
    long tid = syscall(SYS_gettid);
    auto message = interfaces::msg::DynamicSizeArray();
    message.id = count_++;
    message.data.resize(MESSAGE_SIZE);
    RCLCPP_INFO(this->get_logger(), "Publishing Message ID (timer_callback2 tid=%ld): '%ld'", tid, message.id);
    publisher_->publish(std::move(message));
  }

  void subscription_callback(const interfaces::msg::DynamicSizeArray::SharedPtr msg) const {
    long tid = syscall(SYS_gettid);
    RCLCPP_INFO(this->get_logger(), "I heard message ID (subscription_callback tid=%ld): '%ld'", tid, msg->id);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::Publisher<interfaces::msg::DynamicSizeArray>::SharedPtr publisher_;
  rclcpp::Subscription<interfaces::msg::DynamicSizeArray>::SharedPtr subscription_;

  // Need to be stored not to be destructed
  rclcpp::CallbackGroup::SharedPtr group1_;
  rclcpp::CallbackGroup::SharedPtr group2_;
  rclcpp::CallbackGroup::SharedPtr group3_;

  size_t count_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(SampleNode)

/*
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SampleNode>();
  std::vector<std::thread> threads;
  std::vector<rclcpp::executors::SingleThreadedExecutor::SharedPtr> executors;

  node->for_each_callback_group([&node, &executors](rclcpp::CallbackGroup::SharedPtr group) {
      auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor->add_callback_group(group, node->get_node_base_interface());
      executors.push_back(executor);
  });

  for (auto &executor : executors) {
    threads.emplace_back([&executor]() {
        auto tid = syscall(SYS_gettid);
        std::cout << "executor thread (tid=" << tid << ")" << std::endl;
        executor->spin();
    });
  }

  for (auto &t : threads) {
    t.join();
  }

  rclcpp::shutdown();
  return 0;
}
*/
