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

add_executable(signal_filter src/signal_filter.cpp)
target_include_directories(signal_filter PRIVATE include)
ament_target_dependencies(signal_filter rclcpp std_msgs)

install(
  TARGETS
    signal_generator
    signal_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # Source files carry an Apache-2.0 header; cpplint is kept disabled because
  # its legacy ROS profile conflicts with the repository's clang-format file.
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_filters test/test_filters.cpp)
  target_include_directories(test_filters PRIVATE include)
endif()

ament_export_include_directories(include)
ament_package()
