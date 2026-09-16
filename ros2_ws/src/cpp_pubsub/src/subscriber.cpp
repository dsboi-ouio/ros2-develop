#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class LearningSubscriber : public rclcpp::Node
{
public:
  LearningSubscriber()
  : Node("learning_subscriber")// 创建订阅节点
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "learning_topic",// 创建订阅器
      10,
      [this](const std_msgs::msg::String & message)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Received: '%s'",
          message.data.c_str());// 回调函数
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LearningSubscriber>());
  rclcpp::shutdown();
  return 0;
}
