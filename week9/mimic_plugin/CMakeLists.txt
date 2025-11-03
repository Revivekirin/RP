cmake_minimum_required(VERSION 3.5)
project(mimic_plugin)

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

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(mimic_joint_plugin SHARED src/mimic_joint_plugin.cpp)
ament_target_dependencies(mimic_joint_plugin gazebo_dev)
target_link_libraries(mimic_joint_plugin ${GAZEBO_LIBRARIES})

install(TARGETS mimic_joint_plugin
  LIBRARY DESTINATION lib
)


ament_package()
