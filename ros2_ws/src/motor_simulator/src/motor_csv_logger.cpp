#include <chrono>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace motor_simulator
{

class MotorCsvLogger : public rclcpp::Node
{
public:
  MotorCsvLogger()
  : Node("motor_csv_logger")
  {
    const auto output_path = declare_parameter<std::string>("output_path", "motor_response.csv");
    output_.open(output_path, std::ios::out | std::ios::trunc);
    if (!output_.is_open()) {
      throw std::runtime_error("cannot open CSV output: " + output_path);
    }
    output_ << "time_s,position_rad,velocity_rad_s,control_torque_nm\n";
    output_ << std::fixed << std::setprecision(8);
    start_time_ = now();
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "state", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        if (message->position.empty() || message->velocity.empty()) {
          return;
        }
        const double effort = message->effort.empty() ? 0.0 : message->effort.front();
        output_ << (now() - start_time_).seconds() << ',' << message->position.front() << ','
                << message->velocity.front() << ',' << effort << '\n';
        output_.flush();
      });
    RCLCPP_INFO(get_logger(), "Writing motor state to %s", output_path.c_str());
  }

private:
  std::ofstream output_;
  rclcpp::Time start_time_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

}  // namespace motor_simulator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<motor_simulator::MotorCsvLogger>());
  rclcpp::shutdown();
  return 0;
}
