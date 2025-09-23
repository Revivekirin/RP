cmake_minimum_required(VERSION 3.5)
project(action_exam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Primenumber.action"
  DEPENDENCEIS action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(action_exam_server src/action_exam_server.cpp)
ament_target_dependencies(action_exam_server rclcpp rclcpp_action)
add_dependencies(action_exam_server ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_target_interfaces(action_exam_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(action_exam_client src/action_exam_client.cpp)
ament_target_dependencies(action_exam_client rclcpp rclcpp_action)
add_dependencies(action_exam_client ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_target_interfaces(action_exam_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS 
  action_exam_server
  action_exam_client
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
