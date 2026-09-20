cmake_minimum_required(VERSION 3.8)
project(pid_motor_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pid_controller_node src/pid_controller_node.cpp)
target_include_directories(pid_controller_node PRIVATE include)
ament_target_dependencies(pid_controller_node rclcpp std_msgs)

add_executable(angle_pid_controller_node src/angle_pid_controller_node.cpp)
target_include_directories(angle_pid_controller_node PRIVATE include)
ament_target_dependencies(angle_pid_controller_node rclcpp std_msgs)

add_executable(reference_generator_node src/reference_generator_node.cpp)
ament_target_dependencies(reference_generator_node rclcpp std_msgs)

install(
  TARGETS
    pid_controller_node
    angle_pid_controller_node
    reference_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
