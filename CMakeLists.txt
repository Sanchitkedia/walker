cmake_minimum_required(VERSION 3.8)
project(walker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(turtlebot3_gazebo REQUIRED)

# obstacle avoidance
add_executable(WALKER src/walker_node.cpp)
ament_target_dependencies(WALKER rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  WALKER
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}
)

ament_package()