cmake_minimum_required(VERSION 3.8)
project(gimbal_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies (packages)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge)
find_package(OpenCV)

add_library(cv_image_process src/programs/cv_image_process.cpp)
ament_target_dependencies(cv_image_process rclcpp geometry_msgs sensor_msgs cv_bridge OpenCV)

# find dependencies (messages)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

#add executables
add_executable(camera_cv_node src/camera_cv_node.cpp)
#add_executable(gimbal_controller_node src/gimbal_controller_node.cpp)

# target dependencies

## camera_cv_node
target_link_libraries(camera_cv_node cv_image_process)
ament_target_dependencies(camera_cv_node rclcpp geometry_msgs sensor_msgs cv_bridge OpenCV)

## gimbal_controller_node
#ament_target_dependencies(gimbal_controller_node rclcpp geometry_msgs sensor_msgs)

#install
install(TARGETS
  camera_cv_node
  #gimbal_controller_node
  DESTINATION lib/${PROJECT_NAME})

#Install launch files
#install(DIRECTORY launch/
# DESTINATION share/${PROJECT_NAME}/)

ament_package()