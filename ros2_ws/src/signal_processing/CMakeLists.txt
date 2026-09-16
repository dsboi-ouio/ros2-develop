cmake_minimum_required(VERSION 3.8)
project(signal_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(signal_generator src/signal_generator.cpp)
ament_target_dependencies(signal_generator rclcpp std_msgs)

install(
  TARGETS
    signal_generator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
