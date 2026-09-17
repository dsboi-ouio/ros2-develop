cmake_minimum_required(VERSION 3.8)
project(motor_simulator)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(motor_model INTERFACE)
target_include_directories(motor_model INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motor_model INTERFACE cxx_std_17)

add_executable(motor_simulator_node src/motor_simulator_node.cpp)
target_link_libraries(motor_simulator_node motor_model)
ament_target_dependencies(motor_simulator_node rclcpp sensor_msgs std_msgs)

install(TARGETS motor_simulator_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
