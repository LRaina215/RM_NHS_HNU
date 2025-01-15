cmake_minimum_required(VERSION 3.10)
project(example_topic_rclcpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmctrl_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)

# add executable
add_executable(topic_subscribe_01 src/tp_01.cpp)

# specify dependencies
ament_target_dependencies(topic_subscribe_01 rclcpp std_msgs rm_interfaces)

# install target
install(TARGETS
  topic_subscribe_01
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

