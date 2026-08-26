cmake_minimum_required(VERSION 3.16)
project(lidar_imu_odometry)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common registration)
find_package(Eigen3 REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(lidar_imu_odometry_node
  src/lidar_imu_odometry_node.cpp
)

ament_target_dependencies(
  lidar_imu_odometry_node
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  geometry_msgs
)

target_link_libraries(
  lidar_imu_odometry_node
  ${PCL_LIBRARIES}
)

install(
  TARGETS lidar_imu_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
