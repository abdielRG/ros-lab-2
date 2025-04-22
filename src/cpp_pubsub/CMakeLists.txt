cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# executables for talker
add_executable(ddsPubSub src/diff_drive_sim_pub_sub.cpp)
ament_target_dependencies(ddsPubSub rclcpp std_msgs geometry_msgs sensor_msgs)

# help ros2 find executable
install(TARGETS
	ddsPubSub
	DESTINATION lib/${PROJECT_NAME})

ament_package()
