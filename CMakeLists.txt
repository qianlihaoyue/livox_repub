cmake_minimum_required(VERSION 3.5)
project(livox_repub)

SET(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

set(dependencies
  rclcpp
  pcl_ros
  pcl_conversions
  livox_ros_driver2
)

find_package(PCL REQUIRED COMPONENTS common)
include_directories(${PCL_INCLUDE_DIRS})

add_executable(livox_repub livox_repub.cpp)
target_link_libraries(livox_repub ${PCL_LIBRARIES})
ament_target_dependencies(livox_repub ${dependencies})

# ---------------- Install --------------- #
install(TARGETS livox_repub
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
