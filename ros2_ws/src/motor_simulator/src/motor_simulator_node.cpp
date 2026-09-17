#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

#include "motor_simulator/dc_motor_model.hpp"

using namespace std::chrono_literals;

namespace motor_simulator
{

class MotorSimulatorNode : public rclcpp::Node
{
public:
  MotorSimulatorNode()
  : Node("motor_simulator"),
    simulation_frequency_hz_(declare_parameter<double>("simulation_frequency_hz", 1000.0)),
    publish_frequency_hz_(declare_parameter<double>("publish_frequency_hz", 100.0)),
    model_(
      declare_parameter<double>("inertia", 0.01),
      declare_parameter<double>("damping", 0.1),
      MotorState{
        declare_parameter<double>("initial_position_rad", 0.0),
        declare_parameter<double>("initial_velocity_rad_s", 0.0)})
  {
    if (simulation_frequency_hz_ <= 0.0 || publish_frequency_hz_ <= 0.0) {
      throw std::invalid_argument("frequencies must be greater than zero");
    }

    load_torque_ = declare_parameter<double>("load_torque", 0.0);
    joint_name_ = declare_parameter<std::string>("joint_name", "motor_shaft");

    torque_subscription_ = create_subscription<std_msgs::msg::Float64>(
      "torque_cmd", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {control_torque_ = msg->data;});
    load_subscription_ = create_subscription<std_msgs::msg::Float64>(
      "load_torque", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {load_torque_ = msg->data;});

    state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("state", 10);
    position_publisher_ = create_publisher<std_msgs::msg::Float64>("position", 10);
    velocity_publisher_ = create_publisher<std_msgs::msg::Float64>("velocity", 10);

    const auto simulation_period = std::chrono::duration<double>(1.0 / simulation_frequency_hz_);
    const auto publish_period = std::chrono::duration<double>(1.0 / publish_frequency_hz_);
    simulation_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(simulation_period),
      std::bind(&MotorSimulatorNode::update, this));
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      std::bind(&MotorSimulatorNode::publish_state, this));

    RCLCPP_INFO(
      get_logger(), "Motor simulator ready: J=%.4f, B=%.4f, update=%.1f Hz, publish=%.1f Hz",
      model_.inertia(), model_.damping(), simulation_frequency_hz_, publish_frequency_hz_);
  }

private:
  void update()
  {
    model_.step(control_torque_, load_torque_, 1.0 / simulation_frequency_hz_);
  }

  void publish_state()
  {
    const auto & motor_state = model_.state();

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();
    joint_state.name = {joint_name_};
    joint_state.position = {motor_state.position_rad};
    joint_state.velocity = {motor_state.velocity_rad_s};
    joint_state.effort = {control_torque_};
    state_publisher_->publish(joint_state);

    std_msgs::msg::Float64 position;
    position.data = motor_state.position_rad;
    position_publisher_->publish(position);

    std_msgs::msg::Float64 velocity;
    velocity.data = motor_state.velocity_rad_s;
    velocity_publisher_->publish(velocity);
  }

  double simulation_frequency_hz_;
  double publish_frequency_hz_;
  DcMotorModel model_;
  double control_torque_{0.0};
  double load_torque_{0.0};
  std::string joint_name_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr torque_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr load_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace motor_simulator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<motor_simulator::MotorSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
