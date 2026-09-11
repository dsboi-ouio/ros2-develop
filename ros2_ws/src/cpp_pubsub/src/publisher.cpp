#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class LearningPublisher : public rclcpp::Node
{
public:
  LearningPublisher()
  : Node("learning_publisher"), count_(1)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "learning_topic", 10);

    timer_ = this->create_wall_timer(
      1s, [this]() {publish_message();});
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
      message.data.c_str());

    publisher_->publish(message);
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
