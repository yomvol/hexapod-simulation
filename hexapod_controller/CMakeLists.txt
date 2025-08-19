cmake_minimum_required(VERSION 3.5)

project(hexapod_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(hexapod_controller 
  src/hexapod_controller_node.cpp 
  include/hexapod_controller_node.hpp
  src/gait_engine.cpp 
  include/gait_engine.hpp
)
add_executable(leg_workspace_visualizer 
  src/leg_workspace_visualizer_node.cpp 
  src/gait_engine.cpp 
  include/gait_engine.hpp
)

target_include_directories(hexapod_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIR}
)
target_include_directories(leg_workspace_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(hexapod_controller
  rclcpp rclcpp_action control_msgs sensor_msgs trajectory_msgs std_msgs std_srvs visualization_msgs tf2_ros tf2_geometry_msgs tf2_eigen geometry_msgs)
install(TARGETS hexapod_controller DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

ament_target_dependencies(leg_workspace_visualizer
  rclcpp visualization_msgs tf2_ros tf2_geometry_msgs)
install(TARGETS leg_workspace_visualizer DESTINATION lib/${PROJECT_NAME})

ament_package()
