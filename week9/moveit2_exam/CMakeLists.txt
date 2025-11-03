cmake_minimum_required(VERSION 3.5)
project(moveit2_exam)

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
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(rviz2 REQUIRED)
find_package(xacro REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY srdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY moveit_config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY controller
  DESTINATION share/${PROJECT_NAME}
)

add_executable(moveit2_practice_sample src/moveit2_practice_sample.cpp)
ament_target_dependencies(moveit2_practice_sample
    rclcpp
    moveit_ros_planning_interface
    tf2_geometry_msgs
)

install(TARGETS
  moveit2_practice_sample
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
