cmake_minimum_required(VERSION 3.8)
project(drone_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(drone_sim src/drone_sim.cpp)

ament_target_dependencies(
  drone_sim
  rclcpp
  sensor_msgs
)

install(TARGETS
  drone_sim
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

