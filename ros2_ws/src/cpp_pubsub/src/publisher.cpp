#include <chrono>// 处理时间，后面会用到 1s
#include <memory>// 使用智能指针和 std::make_shared
#include <string>

#include "rclcpp/rclcpp.hpp"// ROS 2 的 C++ 客户端库
#include "std_msgs/msg/string.hpp"// ROS 2 提供的字符串消息类型

using namespace std::chrono_literals;// 让 1s 表示1秒

class LearningPublisher : public rclcpp::Node
{
public:
  LearningPublisher()
  : Node("learning_publisher"), count_(1)// 创建发布节点
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "learning_topic", 10);// 创建发布器

    timer_ = this->create_wall_timer(
      1s, [this]() {publish_message();});// 创建定时器
  }

private:
  void publish_message()
  {
    auto message = std_msgs::msg::String();

    message.data =
      "dsboi-ouio is learning ROS 2. Message #" +
      std::to_string(count_++);

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing: '%s'",
      message.data.c_str());// 用 Logger 把“准备发布什么内容”显示在终端中

    publisher_->publish(message);//  把消息发布到 ROS 2 话题上
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LearningPublisher>());
  rclcpp::shutdown();
  return 0;
}
