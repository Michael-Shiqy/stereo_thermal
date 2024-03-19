cmake_minimum_required(VERSION 3.8)
project(pc_sub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)

# add executables
add_executable(pc_sub src/pc_sub.cpp)
ament_target_dependencies(pc_sub rclcpp sensor_msgs pcl_conversions)
target_link_libraries(pc_sub ${PCL_LIBRARIES})

# installation
install(TARGETS pc_sub DESTINATION lib/${PROJECT_NAME})

ament_package()
