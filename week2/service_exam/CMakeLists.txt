cmake_minimum_required(VERSION 3.5)
project(service_exam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MyService.srv"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(service_exam_server src/service_exam_server.cpp)
ament_target_dependencies(service_exam_server rclcpp)
# 인터페이스가 먼저 생성되도록 의존성
add_dependencies(service_exam_server ${PROJECT_NAME}__rosidl_generator_cpp)
# Foxy에 맞는 타입서포트 연결
rosidl_target_interfaces(service_exam_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(service_exam_client src/service_exam_client.cpp)
ament_target_dependencies(service_exam_client rclcpp)
add_dependencies(service_exam_client ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_target_interfaces(service_exam_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  service_exam_server
  service_exam_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

