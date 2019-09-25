cmake_minimum_required(VERSION 3.5)

project(vino_integration)

# Default to C++14                                                              
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14) 
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(object_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

#install(DIRECTORY launch DESTINATION share/${PROJRCT_NAME})

add_executable(vino_integration src/vino_integration.cpp)
ament_target_dependencies(vino_integration
    "rclcpp"
    "std_msgs"
    "object_msgs")
  install(TARGETS vino_integration
    DESTINATION lib/${PROJECT_NAME})

ament_package()