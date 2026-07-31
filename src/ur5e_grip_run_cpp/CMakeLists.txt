cmake_minimum_required(VERSION 3.8)
project(ur5e_grip_run_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robotiq_2f_urcap_adapter REQUIRED)

add_executable(ik_commander src/joint_commander_ik.cpp)

ament_target_dependencies(ik_commander
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_msgs
  rclcpp_action
  robotiq_2f_urcap_adapter
)

target_link_libraries(ik_commander
  Eigen3::Eigen
  yaml-cpp
)

install(TARGETS ik_commander
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()